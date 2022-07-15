// #############################################
/*
 * This file is part of the HA_smarkant project
 *
 * (c) 2021 Leon Schneider, www.leon-schneider.de
 *
 * Based on the Smarkant project from Dirk Grappendorf
 * https://github.com/grappendorf/smarkant
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "config.h"

const int PIN_STATUS_LED = 2;
const int JSON_BUFFER_LENGTH = 512;
const int NUM_POSITION_BUTTONS = 2;
const uint16_t HEIGHT_MAX = 6000;
const uint16_t HEIGHT_MIN = 500;
const unsigned long WAIT_FOR_I2C_BYTES_TIMEOUT_MS = 1000;
const uint8_t I2C_ADDRESS = 0x10;
const unsigned long SERIAL_BAUD_RATE = 9600;
const int PUBLISH_HIGHT_INTERVAL = 30000;

enum I2CCommand {
  I2C_CMD_NOOP,
  I2C_CMD_MOVE_STOP,
  I2C_CMD_MOVE_UP,
  I2C_CMD_MOVE_DOWN,
  I2C_CMD_MOVE_HEIGHT,
  I2C_CMD_MOVE_POSITION,
  I2C_CMD_STORE_POSITION,
  I2C_CMD_STORE_CURRENT_POSITION,
  I2C_CMD_STORE_THRESHOLD,
  I2C_CMD_READ_HEIGHT,
  I2C_CMD_READ_HEIGHT_THRESHOLD,
  I2C_CMD_READ_POSITIONS
};

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup();
void setupWiFi();
void loop();
void setupMqtt();
bool waitForI2CBytesAvailable(int waitForNumBytess);
void callback(char* topic, byte* payload, unsigned int length);
void publishHight();
void reconnect();
void tableStop();
void tableMoveUp();
void tableMoveDown();
void tableMoveToPosition(int position);
void tableMoveToHeight(int height);
void log(const char *str, ...);

long lastActionTime = -1;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  log("Smarkant ready...");

  Wire.begin();

  setupWiFi();
  setupMqtt();  
}

void setupWiFi() {
  log("Connecting to WiFi...");
  pinMode(PIN_STATUS_LED, OUTPUT);
  WiFi.config(local_IP, primaryDNS, gateway, subnet);
  WiFi.begin(WLAN_SSID, WLAN_PASSPHRASE);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(PIN_STATUS_LED, digitalRead(PIN_STATUS_LED) == LOW ? HIGH : LOW);
    log("Failed to connect, try again...");
    delay(100);
  }
  digitalWrite(PIN_STATUS_LED, HIGH);
}

void loop() {
  mqttClient.loop();
  long currentMillis = millis();
  if(lastActionTime<(currentMillis-PUBLISH_HIGHT_INTERVAL)){
    lastActionTime = currentMillis;
    publishHight();
  }
}

void setupMqtt() {
  log("Connecting to MQTT-Broker...");
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(callback);
  reconnect();
}

bool waitForI2CBytesAvailable(int waitForNumBytess) {
  unsigned long waitForI2CBytesTimeout = millis() + WAIT_FOR_I2C_BYTES_TIMEOUT_MS;
  while (Wire.available() < waitForNumBytess) {
    if (millis() > waitForI2CBytesTimeout) {
      return false;
    }
  }
  return true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  log("Message arrived...");
  log((char*)payload);
  // Topic: smarkant/move
  if (strcmp(topic,"smarkant/move")==0){
    if( strstr((char*)payload,"stop") ){
      tableStop();
    }else{
      DynamicJsonDocument json(JSON_BUFFER_LENGTH);
      auto error = deserializeJson(json, (char*)payload);
      if (!error) {
        if (json.containsKey("height")) {
          tableMoveToHeight((int) json["height"]);
        }else if (json.containsKey("position")) {
          tableMoveToPosition((int) json["position"]);
        }
      }
    }
    publishHight();
  }
  // Topic: smarkant/getHeight
  if (strcmp(topic,"smarkant/getHeight")==0){
    publishHight();
  }
  // Topic: smarkant/getConfig
  if (strcmp(topic,"smarkant/getConfig")==0){
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(I2C_CMD_READ_HEIGHT_THRESHOLD);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS, 2);
    if (waitForI2CBytesAvailable(2)) {
      String tmp_str = String(Wire.read() + (Wire.read() << 8));
      char tmp_char[50];
      tmp_str.toCharArray(tmp_char, tmp_str.length() + 1);
      mqttClient.publish("smarkant/config", tmp_char);
    }
  }
  // Topic: smarkant/setConfig
  if (strcmp(topic,"smarkant/setConfig")==0){
    DynamicJsonDocument json(JSON_BUFFER_LENGTH);
    auto error = deserializeJson(json, (char*)payload);
    if (!error) {
      if (json.containsKey("threshold")) {
        int threshold = json["threshold"];
        uint8_t data[] = {I2C_CMD_STORE_THRESHOLD, (uint8_t) (threshold & 0xff), (uint8_t) (threshold >> 8)};
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(data, 3);
        Wire.endTransmission();
      }
    }
  }
  // Topic: smarkant/getPositions
  if (strcmp(topic,"smarkant/getPositions")==0){
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(I2C_CMD_READ_POSITIONS);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS, 2 * NUM_POSITION_BUTTONS);
    if (waitForI2CBytesAvailable(2)) {
      //StaticJsonDocument<JSON_BUFFER_LENGTH> jsonBuffer;
      //JsonObject &json = jsonBuffer.createNestedObject();
      DynamicJsonDocument json(JSON_BUFFER_LENGTH);
      uint16_t position = Wire.read() + (Wire.read() << 8);
      json["position0"] = position;
      position = Wire.read() + (Wire.read() << 8);
      json["position1"] = position;
      String tmp_str;
      serializeJson(json, tmp_str);
      char tmp_char[50];
      tmp_str.toCharArray(tmp_char, tmp_str.length() + 1);
      mqttClient.publish("smarkant/positions", tmp_char);
    }
  }
  // Topic: smarkant/setPositions
  if (strcmp(topic,"smarkant/setPositions")==0){
    log((char*)payload);
    DynamicJsonDocument json(JSON_BUFFER_LENGTH);
    auto error = deserializeJson(json, (char*)payload);
    if (!error) {
      char * attrName = "positionX";
      for (int i = 0; i < NUM_POSITION_BUTTONS; ++i) {
        sprintf(attrName, "position%1d", i);
        if (json.containsKey(attrName)) {
          int position = json[attrName];
          uint8_t data[] = {I2C_CMD_STORE_POSITION, i, (uint8_t) (position & 0xff), (uint8_t) (position >> 8)};
          Wire.beginTransmission(I2C_ADDRESS);
          Wire.write(data, 3);
          Wire.endTransmission();
        }
      }
    }
  }
  // Topic: smarkant/stop
  if (strcmp(topic,"smarkant/stop")==0){
    tableStop();
    publishHight();
  }
  // Topic: smarkant/up
  if (strcmp(topic,"smarkant/up")==0){
    tableMoveUp();
    publishHight();
  }
  // Topic: smarkant/down
  if (strcmp(topic,"smarkant/down")==0){
    tableMoveDown();
    publishHight();
  }
}

void publishHight() {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(I2C_CMD_READ_HEIGHT);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDRESS, 2);
  if (waitForI2CBytesAvailable(2)) {
    uint16_t height = Wire.read() + (Wire.read() << 8);
    String tmp_str = String(height);
    char tmp_char[4];
    tmp_str.toCharArray(tmp_char, tmp_str.length() + 1);
    mqttClient.publish("smarkant/height", tmp_char);
    
    uint16_t heightcm = map(height, 750, 4565, 71, 109);
    tmp_str = String(heightcm);
    tmp_str.toCharArray(tmp_char, tmp_str.length() + 1);
    mqttClient.publish("smarkant/heightcm", tmp_char);
  }
}

void reconnect() {
  while (!mqttClient.connected()) {
    log("Attempting MQTT connection...");
    String clientId = "Smarkant";
    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      log("Connected to mqtt-broker");
      mqttClient.publish("smarkant/status", "online");
      mqttClient.subscribe("smarkant/move");
      mqttClient.subscribe("smarkant/getHeight");
      mqttClient.subscribe("smarkant/getConfig");
      mqttClient.subscribe("smarkant/setConfig");
      mqttClient.subscribe("smarkant/getPositions");
      mqttClient.subscribe("smarkant/setPositions");
      mqttClient.subscribe("smarkant/stop");
      mqttClient.subscribe("smarkant/up");
      mqttClient.subscribe("smarkant/down");
    } else {
      log("failed, rc=");
      log(mqttClient.state());
      log(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void tableStop() {
  log("Table stop");
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(I2C_CMD_MOVE_STOP);
  Wire.endTransmission();
}

void tableMoveUp() {
  log("Table move up");
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(I2C_CMD_MOVE_UP);
  Wire.endTransmission();
}

void tableMoveDown() {
  log("Table move down");
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(I2C_CMD_MOVE_DOWN);
  Wire.endTransmission();
}

void tableMoveToPosition(int position) {
  if (position >= 1 && position <= NUM_POSITION_BUTTONS) {
    log("Table move to position %d", position);
    uint8_t data[] = {I2C_CMD_MOVE_POSITION, 4 - position};
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(data, 2);
    Wire.endTransmission();
  }
}

void tableMoveToHeight(int height) {
  if (height >= HEIGHT_MIN && height <= HEIGHT_MAX) {
    log("Table move to height %d", height);
    uint8_t data[] = {I2C_CMD_MOVE_HEIGHT, (uint8_t) (height & 0xff), (uint8_t) (height >> 8)};
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(data, 3);
    Wire.endTransmission();
  }
}

/**
 * https://gist.github.com/asheeshr/9004783
 */
void log(const char *str, ...)
{
  const int LOG_BUFFER_LENGTH = 32;
  int i;
  int j;
  int count = 0;
  for(i = 0; str[i] != '\0'; i++) {
    if(str[i]=='%') {
      count++;
    }
  }
  if (count == 0) {
    Serial.println (str);
    return;
  }
  char temp[LOG_BUFFER_LENGTH + 1];
  va_list argv;
  va_start(argv, str);
  for(i = 0, j = 0; str[i] != '\0'; i++)
  {
    if(str[i] == '%')
    {
      temp[j] = '\0';
      Serial.print(temp);
      j = 0;
      temp[0] = '\0';
      switch(str[++i])
      {
        case 'd':
          Serial.print(va_arg(argv, int));
          break;
        case 'l':
          Serial.print(va_arg(argv, long));
          break;
        case 'f':
          Serial.print(va_arg(argv, double));
          break;
        case 'c':
          Serial.print((char)va_arg(argv, int));
          break;
        case 's':
          Serial.print(va_arg(argv, char *));
          break;
        default:
          break;
      };
    }
    else
    {
      temp[j] = str[i];
      j = (j+1) % LOG_BUFFER_LENGTH;
      if(j==0)
      {
        temp[LOG_BUFFER_LENGTH] = '\0';
        Serial.print(temp);
        temp[0] = '\0';
      }
    }
  };
  Serial.println();
}

void logProgress() {
  Serial.print('.');
}

void logProgressEnd() {
  Serial.println();
}
