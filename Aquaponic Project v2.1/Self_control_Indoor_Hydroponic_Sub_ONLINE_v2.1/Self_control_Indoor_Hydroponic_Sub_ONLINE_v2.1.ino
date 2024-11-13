//All library for the system
#define SERIAL Serial
#include "DHT.h"
#define DHTTYPE DHT11  // DHT 11
#define debug Serial
#include "Ultrasonic.h"
#include <Servo.h>
#include "CytronMotorDriver.h"
#include <Wire.h>
#include "rgb_lcd.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <Wire.h>
#include <math.h>
#include <PubSubClient.h>
#include "./sha256.h"
#include "./base64.h"
#include "./parson.h"
#include "./morse_code.h"
#include "./utils.h"
#include "./configure.h"

//I/O pin allocation
#define LightPIN A0
#define DHTPIN A1
#define TriggerRelay A3
#define SubReset A4
#define LED 2
#define UVRelayPIN 3
#define FeederRelayPIN 6
#define WateringRelayPIN 7

//Azure and WIFI Code
bool wifiConnected = false;
bool mqttConnected = false;

String iothubHost;
String deviceId;
String sharedAccessKey;

WiFiSSLClient wifiClient;
PubSubClient* mqtt_client = NULL;

int requestId = 0;
int twinRequestId = -1;

#define TELEMETRY_SEND_INTERVAL 5000  // telemetry data sent every 5 seconds
#define PROPERTY_SEND_INTERVAL 15000  // property data sent every 15 seconds
#define SENSOR_READ_INTERVAL 2500     // read sensors every 2.5 seconds

long lastTelemetryMillis = 0;
long lastPropertyMillis = 0;
long lastSensorReadMillis = 0;
bool feederOn = false;
// die property value
int dieNumberValue = 1;

// Define the number of milliseconds in 4 days
unsigned long fourDaysInMillis = 4L * 24L * 60L * 60L * 1000L;  

unsigned long startTime;

// initialize reset function
void(* resetFunc) (void) = 0;

// grab the current time from internet time service
unsigned long getNow() {
  IPAddress address(129, 6, 15, 28);  // time.nist.gov NTP server
  const int NTP_PACKET_SIZE = 48;
  byte packetBuffer[NTP_PACKET_SIZE];
  WiFiUDP Udp;
  Udp.begin(2390);

  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;  // LI, Version, Mode
  packetBuffer[1] = 0;           // Stratum, or type of clock
  packetBuffer[2] = 6;           // Polling Interval
  packetBuffer[3] = 0xEC;        // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  Udp.beginPacket(address, 123);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();

  // wait to see if a reply is available
  int waitCount = 0;
  while (waitCount < 20) {
    delay(500);
    waitCount++;
    if (Udp.parsePacket()) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      unsigned long secsSince1900 = highWord << 16 | lowWord;

      Udp.stop();
      return (secsSince1900 - 2208988800UL);
    }
  }
  return 0;
}

// IoT Hub MQTT publish topics
static const char IOT_EVENT_TOPIC[] = "devices/{device_id}/messages/events/";
static const char IOT_TWIN_REPORTED_PROPERTY[] = "$iothub/twin/PATCH/properties/reported/?$rid={request_id}";
static const char IOT_TWIN_REQUEST_TWIN_TOPIC[] = "$iothub/twin/GET/?$rid={request_id}";
static const char IOT_DIRECT_METHOD_RESPONSE_TOPIC[] = "$iothub/methods/res/{status}/?$rid={request_id}";

// IoT Hub MQTT subscribe topics
static const char IOT_TWIN_RESULT_TOPIC[] = "$iothub/twin/res/#";
static const char IOT_TWIN_DESIRED_PATCH_TOPIC[] = "$iothub/twin/PATCH/properties/desired/#";
static const char IOT_C2D_TOPIC[] = "devices/{device_id}/messages/devicebound/#";
static const char IOT_DIRECT_MESSAGE_TOPIC[] = "$iothub/methods/POST/#";

// split the connection string into it's composite pieces
void splitConnectionString() {
  String connStr = (String)iotConnStr;
  int hostIndex = connStr.indexOf("HostName=");
  int deviceIdIndex = connStr.indexOf(F(";DeviceId="));
  int sharedAccessKeyIndex = connStr.indexOf(";SharedAccessKey=");
  iothubHost = connStr.substring(hostIndex + 9, deviceIdIndex);
  deviceId = connStr.substring(deviceIdIndex + 10, sharedAccessKeyIndex);
  sharedAccessKey = connStr.substring(sharedAccessKeyIndex + 17);
}

// acknowledge the receipt of a setting back to Azure IoT Central (makes the setting status turn green)
void acknowledgeSetting(const char* propertyKey, const char* propertyValue, int version) {
  // for IoT Central need to return acknowledgement
  const static char* responseTemplate = "{\"%s\":{\"value\":%s,\"statusCode\":%d,\"status\":\"%s\",\"desiredVersion\":%d}}";
  char payload[1024];
  sprintf(payload, responseTemplate, propertyKey, propertyValue, 200, "completed", version);
  Serial.println(payload);
  String topic = (String)IOT_TWIN_REPORTED_PROPERTY;
  char buff[20];
  topic.replace("{request_id}", itoa(requestId, buff, 10));
  mqtt_client->publish(topic.c_str(), payload);
  requestId++;
}

// process direct method requests
void handleDirectMethod(String topicStr, String payloadStr) {
  String msgId = topicStr.substring(topicStr.indexOf("$RID=") + 5);
  String methodName = topicStr.substring(topicStr.indexOf("$IOTHUB/METHODS/POST/") + 21, topicStr.indexOf("/?$"));
  Serial_printf("Direct method call:\n\tMethod Name: %s\n\tParameters: %s\n", methodName.c_str(), payloadStr.c_str());
  if (strcmp(methodName.c_str(), "ECHO") == 0) {
    // acknowledge receipt of the command
    String response_topic = (String)IOT_DIRECT_METHOD_RESPONSE_TOPIC;
    char buff[20];
    response_topic.replace("{request_id}", msgId);
    response_topic.replace("{status}", "200");  //OK
    mqtt_client->publish(response_topic.c_str(), "");

    // output the message as morse code
    JSON_Value* root_value = json_parse_string(payloadStr.c_str());
    JSON_Object* root_obj = json_value_get_object(root_value);
    const char* msg = json_object_get_string(root_obj, "displayedValue");
    morse_encodeAndFlash(msg);
    json_value_free(root_value);
  }
}

// process Cloud to Device (C2D) requests
void handleCloud2DeviceMessage(String topicStr, String payloadStr) {
  Serial_printf("Twin property change:\n\tPayload: %s\n", payloadStr.c_str());
}

// process twin property (settings in IoT Central language) changes
void handleTwinPropertyChange(String topicStr, String payloadStr) {
  // read the property values sent using JSON parser
  JSON_Value* root_value = json_parse_string(payloadStr.c_str());
  JSON_Object* root_obj = json_value_get_object(root_value);
  const char* propertyKey = json_object_get_name(root_obj, 0);
  double propertyValue;
  double version;
  if (strcmp(propertyKey, "fanSpeed") == 0) {
    JSON_Object* valObj = json_object_get_object(root_obj, propertyKey);
    propertyValue = json_object_get_number(valObj, "value");
    version = json_object_get_number(root_obj, "$version");
    char propertyValueStr[8];
    itoa(propertyValue, propertyValueStr, 10);
    Serial_printf("Fan Speed setting change received with value: %s\n", propertyValueStr);
    acknowledgeSetting(propertyKey, propertyValueStr, version);
  }
  json_value_free(root_value);
}

// callback for MQTT subscriptions
void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = (String)topic;
  topicStr.toUpperCase();
  payload[length] = '\0';
  String payloadStr = (String)((char*)payload);

  if (topicStr.startsWith("$IOTHUB/METHODS/POST/")) {  // direct method callback
    handleDirectMethod(topicStr, payloadStr);
  } else if (topicStr.indexOf("/MESSAGES/DEVICEBOUND/") > -1) {  // cloud to device message
    handleCloud2DeviceMessage(topicStr, payloadStr);
  } else if (topicStr.startsWith("$IOTHUB/TWIN/PATCH/PROPERTIES/DESIRED")) {  // digital twin desired property change
    handleTwinPropertyChange(topicStr, payloadStr);
  } else if (topicStr.startsWith("$IOTHUB/TWIN/RES")) {  // digital twin response
    int result = atoi(topicStr.substring(topicStr.indexOf("/RES/") + 5, topicStr.indexOf("/?$")).c_str());
    int msgId = atoi(topicStr.substring(topicStr.indexOf("$RID=") + 5, topicStr.indexOf("$VERSION=") - 1).c_str());
    if (msgId == twinRequestId) {
      // twin request processing
      twinRequestId = -1;
      // output limited to 512 bytes so this output may be truncated
      Serial_printf("Current state of device twin:\n%s", payloadStr.c_str());
      if (length > 512)
        Serial.println(" ...");
      else
        Serial.println();
    } else {
      if (result >= 200 && result < 300) {
        Serial_printf("--> IoT Hub acknowledges successful receipt of twin property: %d\n", msgId);
      } else {
        Serial_printf("--> IoT Hub could not process twin property: %d, error: %d\n", msgId, result);
      }
    }
  } else {  // unknown message
    Serial_printf("Unknown message arrived [%s]\nPayload contains: %s", topic, payloadStr.c_str());
  }
}

// connect to Azure IoT Hub via MQTT
void connectMQTT(String deviceId, String username, String password) {
  mqtt_client->disconnect();

  Serial.println("Starting IoT Hub connection");
  int retry = 0;
  while (retry < 10 && !mqtt_client->connected()) {
    if (mqtt_client->connect(deviceId.c_str(), username.c_str(), password.c_str())) {
      Serial.println("===> mqtt connected");
      mqttConnected = true;
      return;
    } else {
      Serial.print("---> mqtt failed, rc=");
      Serial.println(mqtt_client->state());
      delay(2000);
      retry++;
    }
  }
  Serial_printf("MQTT connection failed after 10 retries.");
  resetFunc();
}

// create an IoT Hub SAS token for authentication
String createIotHubSASToken(char* key, String url, long expire) {
  url.toLowerCase();
  String stringToSign = url + "\n" + String(expire);
  int keyLength = strlen(key);

  int decodedKeyLength = base64_dec_len(key, keyLength);
  char decodedKey[decodedKeyLength];

  base64_decode(decodedKey, key, keyLength);

  Sha256* sha256 = new Sha256();
  sha256->initHmac((const uint8_t*)decodedKey, (size_t)decodedKeyLength);
  sha256->print(stringToSign);
  char* sign = (char*)sha256->resultHmac();
  int encodedSignLen = base64_enc_len(HASH_LENGTH);
  char encodedSign[encodedSignLen];
  base64_encode(encodedSign, sign, HASH_LENGTH);
  delete (sha256);

  return "SharedAccessSignature sr=" + url + "&sig=" + urlEncode((const char*)encodedSign) + "&se=" + String(expire);
}

//DHT11 Variables
DHT dht(DHTPIN, DHTTYPE);  //   DHT11
float temp_hum_val[2] = { 0 };
long TemperatureValue = 0.0;
long HumidityValue = 0.0;
long TemperatureGood = 60;
long TemperatureBad;
long HumidityGood = 80;
long HumidityBad;

//Light Variables
int LightValue;
int LightGood = 80;
int LightBad = 20;
long LightFuzzy;

//Weight for each data in Hidden layer for Fuzzy Logic
float W_Light = 0.6;

//Transmit Data
long Mode = 1;
long Reset = 1;

void DHTActive() {
  temp_hum_val[2] = { 0 };
  if (!dht.readTempAndHumidity(temp_hum_val)) {
    HumidityValue = temp_hum_val[0];
    TemperatureValue = temp_hum_val[1];
  }
}

void LightActive() {
  LightValue = analogRead(LightPIN);
  LightValue = map(LightValue, 0, 800, 0, 100);
}

void UVLightActive(int Mode) {
  if (Mode == 0) {
    digitalWrite(UVRelayPIN, LOW);
  } else if (Mode == 1) {
    digitalWrite(UVRelayPIN, HIGH);
  }
}

void AutoRelay() {
  Mode = digitalRead(TriggerRelay);  // receive byte as an integer
  if (Mode > 0) {
    //digitalWrite(FeederRelayPIN, HIGH);
    digitalWrite(WateringRelayPIN, HIGH);
    Serial.println("All Relay ON");
  }

  else {
    //digitalWrite(FeederRelayPIN, LOW);
    digitalWrite(WateringRelayPIN, LOW);
    Serial.println("All Relay OFF");
  }
}


void CollectData() {
  DHTActive();
  LightActive();
}

void AutoUV() {
  if (LightValue < 20) {
    UVLightActive(1);
    Serial.println("Light ON");
  } else if (LightValue >= 30) {
    UVLightActive(0);
    Serial.println("Light 0FF");
  }
}

void Transducer() {
  AutoUV();
  AutoRelay();
}

void AZURE() {
  if (mqtt_client->connected()) {
    // give the MQTT handler time to do it's thing
    mqtt_client->loop();
    lastSensorReadMillis = millis();
    if (millis() - lastSensorReadMillis > SENSOR_READ_INTERVAL) {
      CollectData();
      lastSensorReadMillis = millis();
    }

    // send telemetry values every 5 seconds
    if (millis() - lastTelemetryMillis > TELEMETRY_SEND_INTERVAL) {
      Serial.println("Sending telemetry ...");
      String topic = (String)IOT_EVENT_TOPIC;
      topic.replace("{device_id}", deviceId);
      //char buff[10];
      String payload = "{\"LightValue\": {LightValue}, \"TemperatureValue\": {TemperatureValue}, \"HumidityValue\": {HumidityValue}, \"Mode\": {Mode},}";
      payload.replace("{LightValue}", String(LightValue));
      payload.replace("{TemperatureValue}", String(TemperatureValue));
      payload.replace("{HumidityValue}", String(HumidityValue));
      payload.replace("{Mode}", String(Mode));
      Serial_printf("\t%s\n", payload.c_str());
      mqtt_client->publish(topic.c_str(), payload.c_str());
      lastTelemetryMillis = millis();
    }

    // send a property update every 15 seconds
    if (millis() - lastPropertyMillis > PROPERTY_SEND_INTERVAL) {
      Serial.println("Sending digital twin property ...");

      String topic = (String)IOT_TWIN_REPORTED_PROPERTY;
      char buff[20];
      topic.replace("{request_id}", itoa(requestId, buff, 10));
      String payload = "{\"dieNumber\": {dieNumberValue}}";
      payload.replace("{dieNumberValue}", itoa(dieNumberValue, buff, 10));

      mqtt_client->publish(topic.c_str(), payload.c_str());
      requestId++;

      lastPropertyMillis = millis();
    }
  } else {
    ConnAzure();
  }
}

void ConnAzure() {
  splitConnectionString();

  // create SAS token and user name for connecting to MQTT broker
  String url = iothubHost + urlEncode(String("/devices/" + deviceId).c_str());
  char* devKey = (char*)sharedAccessKey.c_str();
  long expire = getNow() + 864000;

  if (!mqtt_client->connected()) {
      Serial_printf("MQTT Disconnected, trying to reconnect...");

      String sasToken = createIotHubSASToken(devKey, url, expire);
      String username = iothubHost + "/" + deviceId + "/api-version=2016-11-14";

      // connect to the IoT Hub MQTT broker
      wifiClient.connect(iothubHost.c_str(), 8883);
      mqtt_client = new PubSubClient(iothubHost.c_str(), 8883, wifiClient);
      connectMQTT(deviceId, username, sasToken);

      // If still not connected after retries, handle the failure
      if (!mqtt_client->connected()) {
        Serial_printf("Failed to reconnect to MQTT, will retry later.");
        return;  // Exit if unable to connect after retrying
      }

      mqtt_client->setCallback(callback);

      // // add subscriptions
      mqtt_client->subscribe(IOT_TWIN_RESULT_TOPIC);         // twin results
      mqtt_client->subscribe(IOT_TWIN_DESIRED_PATCH_TOPIC);  // twin desired properties
      String c2dMessageTopic = IOT_C2D_TOPIC;
      c2dMessageTopic.replace("{device_id}", deviceId);
      mqtt_client->subscribe(c2dMessageTopic.c_str());   // cloud to device messages
      mqtt_client->subscribe(IOT_DIRECT_MESSAGE_TOPIC);  // direct messages

      // request full digital twin update
      String topic = (String)IOT_TWIN_REQUEST_TWIN_TOPIC;
      char buff[20];
      topic.replace("{request_id}", itoa(requestId, buff, 10));
      twinRequestId = requestId;
      requestId++;
      mqtt_client->publish(topic.c_str(), "");

      // initialize timers
      lastTelemetryMillis = millis();
      lastPropertyMillis = millis();
  }
}

void setup() {
  pinMode(TriggerRelay, INPUT);
  pinMode(SubReset, INPUT);
  Serial.begin(115200);
  // attempt to connect to Wifi network:
  Serial.print("WiFi Firmware version is ");
  Serial.println(WiFi.firmwareVersion());
  int status = WL_IDLE_STATUS;
  Serial_printf("Attempting to connect to Wi-Fi SSID: %s ", wifi_ssid);
  status = WiFi.begin(wifi_ssid, wifi_password);
  if (status != WL_CONNECTED) {
    for (int n = 0; n < 3; n++) {
      status = WiFi.begin(wifi_ssid, wifi_password);
      if (status == WL_CONNECTED)
        break;
      else
        continue;
    }
  }
  else if (status == WL_CONNECTED) {
    Serial.println();
    digitalWrite(LED, HIGH);
    ConnAzure();
  }

  // Initialize NTP client
  timeClient.begin();
  // Set time zone offset for Malaysia (UTC +8)
  timeClient.setTimeOffset(28800);  // Malaysia Time: UTC +8 = 8 * 3600 seconds
}

void CheckFeedingTime() {
	timeClient.update(); // Update the time from NTP server
	
    // Get the current hour, minute, and second from the NTP client
	int currentHour = timeClient.getHours();
	int currentMinute = timeClient.getMinutes();
	int currentSecond = timeClient.getSeconds();
	
	if (currentHour == 8 && currentMinute == 0) {
		if (currentSecond < 6) {
			feederOn = true;
		} else {
			feederOn = false;			
		}
	}
}

void feedingFish() {
	if (feederOn) {
		digitalWrite(FeederRelayPIN, HIGH);
	} else {
		digitalWrite(FeederRelayPIN, LOW);
	}
}

void loop() {
  CheckFeedingTime();
  feedingFish();
  CollectData();
  Transducer();
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifi_ssid, wifi_password);
    // Buat checking wifi attempts, kalau gagal ia cuba connect semula
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
        WiFi.begin(wifi_ssid, wifi_password);
        delay(5000); // wait 5 seconds before retry
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        AZURE(); // Berjaya connect wifi baru execute function
    } else {
        Serial_printf("Failed to reconnect to WiFi"); // Kalau attempt gagal melebihi 5 kali
        resetFunc();
    }
  } else {
    AZURE();
  }

  if (millis() - startTime >= fourDaysInMillis) {
    resetFunc();  // Trigger the reset after 4 days
  }
  delay(1000);
}
