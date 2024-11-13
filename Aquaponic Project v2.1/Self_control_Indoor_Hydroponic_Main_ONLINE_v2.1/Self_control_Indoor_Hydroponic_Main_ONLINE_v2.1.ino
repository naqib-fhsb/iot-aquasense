//All library for the system
#define SERIAL Serial
#define debug Serial
#include "Ultrasonic.h"
#include "CytronMotorDriver.h"
#include <Wire.h>
#include "rgb_lcd.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
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
#define Moisture1PIN A0
#define TurbidityPIN A1
#define Moisture2PIN A2
#define TDSPIN A3
Ultrasonic ultrasonic(2);
#define ButtonA 3
CytronMD motor(PWM_DIR, 5, 4);
#define RelayPIN 6
#define ButtonB 7
#define WaterPIN 8
#define SubReset 9

//Default Value
float MoistureGood = 700;
int MoistureChange = 200;
float TurbidityGood = 2;
int TurbidityChange = 2;
float TDSGood = 300;
int TDSChange = 200;
float WaterMax = 25;
float WaterEmpty = 35;
int page = 1;
int Speed;
char* pump = "Norm";
char* state = "Good";

//Weight for each data Logic
float W_Turbidity = 0.3;
float W_TDS = 0.7;

//Main Board Variables
//Moisture Variables
int MoistureValue;
int Moisture1Value;
int Moisture2Value;
float MoistureFuzzy;

//Turbidity Variables
int TurbiditySensor;
float TurbidityValue = 0.0;
float TurbidityFuzzy;

//TDS Variables
int TDSSensor = 0;
float TDSValue = 0;
float TDSVoltage = 0;
float TDSFuzzy;

//Ultrasonic Variables
float UltrasonicValue = 0.00;
float WaterLevel = 0.0;
float WaterLevelFuzzy;

//WaterPump Variables
const int low = 100;
const int med = 150;
const int high = 200;
const int turbo = 255;

//Water Sensor Variables
int WaterSense = 1;
long WaterSenseFuzzy;

//RGB LCD Backlight Variables
int count = 0;
const int colorR = 255;
const int colorG = 255;
const int colorB = 255;
const int coloroff = 0;
int countperdata = 5;

//Fuzzy Logic Control Variables for Active Transducer
float WaterQuality;
float WaterPlant;
int feederGO = 0.8;
int WaterPlantGO = 0.7;

//Current Transducer State Azure Data
int CurrentSpeed;
int CurrentWaterPump;
int CurrentFeederPower;

//Maintenance Mode Variables
int A;
int B;
int Counter = 0;
int Mode = 0;
int NewDefault;

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
        Serial.println();
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

  int retry = 0;
  while (retry < 10 && !mqtt_client->connected()) {
    if (mqtt_client->connect(deviceId.c_str(), username.c_str(), password.c_str())) {
      mqttConnected = true;
      return;
    } else {
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


rgb_lcd lcd;

//All Codes
void MoistureActive() {
  Moisture1Value = analogRead(Moisture1PIN);
  Moisture2Value = analogRead(Moisture2PIN);
  MoistureValue = (Moisture1Value + Moisture2Value) / 2;
}

void WaterActive() {
  WaterSense = digitalRead(WaterPIN);
}

void TurbidityActive() {
  TurbiditySensor = analogRead(TurbidityPIN);
  TurbidityValue = TurbiditySensor * (5.0 / 1024.0);
}

void TDSActive() {
  TDSSensor = analogRead(TDSPIN);
  TDSVoltage = TDSSensor * 5 / 1024.0;
  TDSValue = (133.42 / TDSVoltage * TDSVoltage * TDSVoltage - 255.86 * TDSVoltage * TDSVoltage + 857.39 * TDSVoltage) * 0.5;
}

void UltrasonicActive() {
  UltrasonicValue = ultrasonic.MeasureInCentimeters();
  delay(100);
  WaterLevel = ((WaterEmpty - UltrasonicValue) / (WaterEmpty - WaterMax)) * 100;
}

void WaterPumpActive(int Speed) {
  if (Speed == 1) {
    motor.setSpeed(low);
    pump = "Slow";
  } else if (Speed == 2) {
    motor.setSpeed(med);
    pump = "Norm";
  } else if (Speed == 3) {
    motor.setSpeed(high);
    pump = "Fast";
  } else if (Speed == 0) {
    motor.setSpeed(0);
    pump = "Off.";
  }
}

void ActiveRelay(int n) {
  if (n == 1) {
    digitalWrite(RelayPIN, HIGH);
  } else if (n == 0) {
    digitalWrite(RelayPIN, LOW);
    state = "Fail";
  }
}


void colorlcd(int color) {
  if (color == 1) {
    lcd.setRGB(colorR, coloroff, coloroff);
  } else if (color == 2) {
    lcd.setRGB(colorR, 100, coloroff);
  } else if (color == 3) {
    lcd.setRGB(coloroff, coloroff, colorB);
  } else if (color == 4) {
    lcd.setRGB(colorR, colorG, colorB);
  }
}

void clearLCD() {
  lcd.setCursor(0, 0);
  lcd.print("                     ");
  lcd.setCursor(0, 1);
  lcd.print("                     ");
}

void CollectData() {
  MoistureActive();
  TurbidityActive();
  TDSActive();
  UltrasonicActive();
  WaterActive();
  A = digitalRead(ButtonA);
  B = digitalRead(ButtonB);
  if (A == 1 && B == 1)
    Counter = 1;
  else
    Counter = 0;
}

void FuzzyControl() {
  TurbidityFuzzy = (((TurbidityChange - TurbidityGood) + TurbidityValue) / (TurbidityChange)) * W_Turbidity;
  TDSFuzzy = (((TDSChange + TDSGood) - TDSValue) / (TDSChange)) * W_TDS;
  WaterQuality = (TDSFuzzy + TurbidityFuzzy) / (W_Turbidity + W_TDS);
  WaterPlant = ((MoistureValue + MoistureChange) - MoistureGood) / (MoistureChange);
}

void LCDDisplay() {
  if (digitalRead(ButtonA) == 1)
    page = 1;
  else if (digitalRead(ButtonB) == 1)
    page = 2;

  if (page == 1) {
    if (WaterQuality > 0.8)
      colorlcd(4);
    else if (WaterQuality <= 0.8 && WaterQuality > 0.5)
      colorlcd(2);
    else if (WaterQuality < 0.5)
      colorlcd(1);
    lcd.setCursor(0, 0);
    lcd.print("AquaPure  : ");
    lcd.setCursor(12, 0);
    lcd.print(state);
    lcd.setCursor(0, 1);
    lcd.print("Volume    : ");
    lcd.setCursor(12, 1);
    lcd.print(WaterLevel);
  }

  else if (page == 2) {
    if (WaterPlant < 0.8 && WaterPlant >= 0.5)
      colorlcd(2);
    else if (WaterPlant < 0.5)
      colorlcd(1);
    else
      colorlcd(4);
    lcd.setCursor(0, 0);
    lcd.print("SoilMoist : ");
    lcd.setCursor(12, 0);
    lcd.print(WaterPlant);
    lcd.setCursor(0, 1);
    lcd.print("Pump      : ");
    lcd.setCursor(12, 1);
    lcd.print(pump);
  }
}

void AutoFilter() {
  clearLCD();
  ActiveRelay(0);
  motor.setSpeed(turbo);
  colorlcd(3);
  lcd.setCursor(0, 0);
  lcd.print("Filtering Water");
  lcd.setCursor(0, 1);
  lcd.print(WaterQuality);
  delay(5000);
  clearLCD();
}


void Transducer() {
  if (WaterQuality > 0.4 && WaterPlant > 0.4) {
    ActiveRelay(1);
    if (WaterQuality < 0.5) {
      WaterPumpActive(3);
      state = "Fail";
    } else if (WaterQuality >= 0.5 && WaterQuality < 0.8) {
      WaterPumpActive(2);
      state = "Warn";
    } else {
      WaterPumpActive(1);
      state = "Good";
    }
  } else
    ActiveRelay(0);
}

void AZURE() {
  if (mqtt_client->connected()) {
    // give the MQTT handler time to do it's thing
    mqtt_client->loop();
    lastSensorReadMillis = millis();
    if (millis() - lastSensorReadMillis > SENSOR_READ_INTERVAL) {
      lastSensorReadMillis = millis();
    }

    // send telemetry values every 5 seconds
    if (millis() - lastTelemetryMillis > TELEMETRY_SEND_INTERVAL) {
      String topic = (String)IOT_EVENT_TOPIC;
      topic.replace("{device_id}", deviceId);
      //char buff[10];
      String payload = "{\"TurbidityValue\": {TurbidityValue}, \"TDSValue\": {TDSValue}, \"WaterQuality\": {WaterQuality}, \"WaterLevel\": {WaterLevel}, \"UltrasonicValue\": {UltrasonicValue}, \"WaterPlant\": {WaterPlant}}";
      payload.replace("{TurbidityValue}", String(TurbidityValue));
      payload.replace("{TDSValue}", String(TDSValue));
      payload.replace("{WaterQuality}", String(WaterQuality));
      payload.replace("{WaterLevel}", String(WaterLevel));
      payload.replace("{UltrasonicValue}", String(UltrasonicValue));
      payload.replace("{WaterPlant}", String(WaterPlant));
      Serial_printf("\t%s\n", payload.c_str());
      mqtt_client->publish(topic.c_str(), payload.c_str());
      lastTelemetryMillis = millis();

      // send a property update every 15 seconds
      if (millis() - lastPropertyMillis > PROPERTY_SEND_INTERVAL) {

        String topic = (String)IOT_TWIN_REPORTED_PROPERTY;
        char buff[20];
        topic.replace("{request_id}", itoa(requestId, buff, 10));
        String payload = "{\"dieNumber\": {dieNumberValue}}";
        payload.replace("{dieNumberValue}", itoa(dieNumberValue, buff, 10));

        mqtt_client->publish(topic.c_str(), payload.c_str());
        requestId++;

        lastPropertyMillis = millis();
      }
    }
  } else {
    ConnAzure();
  }
}

void Maintenance() {
  clearLCD();
  colorlcd(4);
  lcd.setCursor(0, 0);
  lcd.print("Maintenance");
  lcd.setCursor(0, 1);
  lcd.print("Mode");
  delay(2000);
  for (int stay = 1; stay == 1;) {

    if (Mode == 1) {
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print("Turbidity");
      lcd.setCursor(0, 1);
      lcd.print("Checking");
      delay(2000);
      clearLCD();
      for (Mode; Mode == 1;) {
        AZURE();
        TurbidityActive();
        lcd.setCursor(0, 0);
        lcd.print("Default : ");
        lcd.println(TurbidityGood);
        lcd.setCursor(0, 1);
        lcd.print("Current : ");
        lcd.println(TurbidityValue);
        delay(200);
        A = digitalRead(ButtonA);
        B = digitalRead(ButtonB);
        if (A == 1 && B == 0) {
          delay(200);
          TurbidityGood += 0.1;
        } else if (A == 0 && B == 1) {
          delay(200);
          TurbidityGood -= 0.1;
        } else if (A == 1 && B == 1) {
          Mode = 2;
        }
      }
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print("Turbidity");
      lcd.setCursor(0, 1);
      lcd.print("Updated");
      delay(2000);
      clearLCD();
    }

    else if (Mode == 2) {
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print("TDS Sensor");
      lcd.setCursor(0, 1);
      lcd.print("Checking");
      delay(2000);
      clearLCD();
      for (Mode; Mode == 2;) {
        AZURE();
        TDSActive();
        lcd.setCursor(0, 0);
        lcd.print("Default : ");
        lcd.println(TDSGood);
        lcd.setCursor(0, 1);
        lcd.print("Current : ");
        lcd.println(TDSValue);
        delay(200);
        A = digitalRead(ButtonA);
        B = digitalRead(ButtonB);
        if (A == 1 && B == 0) {
          delay(200);
          TDSGood += 10;
        } else if (A == 0 && B == 1) {
          delay(200);
          TDSGood -= 10;
        } else if (A == 1 && B == 1) {
          Mode = 3;
        }
      }
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print("TDS Sensor");
      lcd.setCursor(0, 1);
      lcd.print("Updated");
      delay(2000);
      clearLCD();
    } else if (Mode == 3) {
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print("Water Level");
      lcd.setCursor(0, 1);
      lcd.print("Checking");
      delay(2000);
      clearLCD();
      for (Mode; Mode == 3;) {
        AZURE();
        UltrasonicActive();
        lcd.setCursor(0, 0);
        lcd.print("Default : ");
        lcd.println(WaterMax);
        lcd.setCursor(0, 1);
        lcd.print("Current : ");
        lcd.println(UltrasonicValue);
        delay(200);
        A = digitalRead(ButtonA);
        B = digitalRead(ButtonB);
        if (A == 1 && B == 0) {
          delay(200);
          WaterMax += 0.1;
        } else if (A == 0 && B == 1) {
          delay(200);
          WaterMax -= 0.1;
        } else if (A == 1 && B == 1) {
          Mode = 4;
        }
      }
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print("Water Level");
      lcd.setCursor(0, 1);
      lcd.print("Updated");
      delay(2000);
      clearLCD();
    }

    else if (Mode == 4) {
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print("Moisture");
      lcd.setCursor(0, 1);
      lcd.print("Checking");
      delay(2000);
      clearLCD();
      for (Mode; Mode == 4;) {
        AZURE();
        MoistureActive();
        lcd.setCursor(0, 0);
        lcd.print("Default : ");
        lcd.println(MoistureGood);
        lcd.setCursor(0, 1);
        lcd.print("Current : ");
        lcd.println(MoistureValue);
        delay(200);
        A = digitalRead(ButtonA);
        B = digitalRead(ButtonB);
        if (A == 1 && B == 0) {
          delay(200);
          MoistureGood += 10;
        } else if (A == 0 && B == 1) {
          delay(200);
          MoistureGood -= 10;
        } else if (A == 1 && B == 1) {
          Mode = 5;
        }
      }
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print("Moisture");
      lcd.setCursor(0, 1);
      lcd.print("Updated");
      delay(2000);
      clearLCD();
    }

    else if (Mode == 5) {
      Mode = 0;
      Counter = 0;
      stay = 0;
      clearLCD();
      lcd.setCursor(0, 0);
      lcd.print("Exit");
      delay(2000);
    }
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
  Serial.begin(115200);
  lcd.begin(16, 2);
  pinMode(ButtonA, INPUT);
  pinMode(ButtonB, INPUT);
  lcd.setCursor(0, 0);
  motor.setSpeed(turbo);
  CollectData();
  TurbidityGood = TurbidityValue;
  TDSGood = TDSValue;
  WaterMax = UltrasonicValue;
  MoistureGood = MoistureValue;
  lcd.print("Connect to Wi-Fi");

  // attempt to connect to Wifi network:
  int status = WL_IDLE_STATUS;
  Serial_printf("Attempting to connect to Wi-Fi SSID: %s ", wifi_ssid);
  status = WiFi.begin(wifi_ssid, wifi_password);
  if (status != WL_CONNECTED) {
    for (int n=0;n<3;n++) {
      status = WiFi.begin(wifi_ssid, wifi_password);
      if (status == WL_CONNECTED)
      break;
      else
      continue;
    }
  }

  else if (status == WL_CONNECTED) {
  lcd.setCursor(0, 1);
  lcd.print("                  ");
  lcd.setCursor(0, 1);
  lcd.print("Connected");
  startTime = millis();
  ConnAzure();
  }

  clearLCD();
  lcd.setCursor(0, 0);
  lcd.print("IoT Azure");
  lcd.setCursor(0, 1);
  lcd.print("AquaSense");
  delay(2000);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
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
  if (Counter < 1) {
    CollectData();
    FuzzyControl();
    if (WaterQuality < 0.4)
      AutoFilter();
    else if (WaterQuality >= 0.4) {
      if (WaterSense == 1) {
        delay(500);
        Transducer();
        LCDDisplay();
      } else if (WaterSense == 0) {
        clearLCD();
        colorlcd(1);
        lcd.setCursor(0, 0);
        lcd.print("Filter Overflow");
        WaterPumpActive(0);
        delay(500);
        ActiveRelay(0);
      }
    }
  } else {
    Mode = 1;
    Maintenance();
  }
  if (millis() - startTime >= fourDaysInMillis) {
    resetFunc();  // Trigger the reset after 4 days
  }
  delay(1000);
}
