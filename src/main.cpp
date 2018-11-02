#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <Roomba.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
extern "C" {
#include "user_interface.h"
}

// Remote debugging over telnet. Just run:
// `telnet roomba.local` OR `nc roomba.local 23`
#if LOGGING
#include <RemoteDebug.h>
#define DLOG(msg, ...) if(Debug.isActive(Debug.DEBUG)){Debug.printf(msg, ##__VA_ARGS__);}
RemoteDebug Debug;
#else
#define DLOG(msg, ...)
#endif

// Roomba setup
Roomba roomba(&Serial, Roomba::Baud115200);

// Roomba state
bool cleaning = false;
bool docked = false;

// Network setup
WiFiClient wifiClient;

// MQTT setup
PubSubClient mqttClient(wifiClient);
const PROGMEM char *commandTopic = MQTT_COMMAND_TOPIC;
const PROGMEM char *statusTopic = MQTT_STATE_TOPIC;

void wakeOnDock(void) {
  DLOG("Wakeup Roomba on dock\n");
  pinMode(BRC_PIN,OUTPUT);
  digitalWrite(BRC_PIN,LOW);
  delay(200);
  pinMode(BRC_PIN,INPUT);
  delay(200);
  Serial.write(128); // Start
#ifdef ROOMBA_650_SLEEP_FIX
  // Some black magic from @AndiTheBest to keep the Roomba awake on the dock
  // See https://github.com/johnboiles/esp-roomba-mqtt/issues/3#issuecomment-402096638
  delay(10);
  Serial.write(135); // Clean
  delay(150);
  Serial.write(143); // Dock
#endif
}

void wakeOffDock(void) {
  DLOG("Wakeup Roomba off Dock\n");
  Serial.write(131); // Safe mode
  delay(300);
  Serial.write(130); // Passive mode
}

bool performCommand(const char *cmdchar) {
  // Char* string comparisons dont always work
  String cmd(cmdchar);

  // MQTT protocol commands
  if (cmd == "turn_on") {
    DLOG("Turning on\n");
    roomba.cover();
    cleaning = true;
  } else if (cmd == "turn_off") {
    DLOG("Turning off\n");
    roomba.power();
    cleaning = false;
  } else if (cmd == "toggle") {
    DLOG("Toggling\n");
    roomba.cover();
  } else if (cmd == "stop") {
    if (cleaning) {
      DLOG("Stopping\n");
      roomba.cover();
    } else {
      DLOG("Not cleaning, can't stop\n");
    }
  } else if (cmd == "clean_spot") {
    DLOG("Cleaning Spot\n");
    cleaning = true;
    roomba.spot();
  } else if (cmd == "locate") {
    DLOG("Locating\n");
    // TODO
  } else if (cmd == "return_to_base") {
    DLOG("Returning to Base\n");
    cleaning = true;
    roomba.dock();
  } else {
    return false;
  }
  return true;
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  DLOG("Received mqtt callback for topic %s\n", topic);
  if (strcmp(commandTopic, topic) == 0) {
    // turn payload into a null terminated string
    char *cmd = (char *)malloc(length + 1);
    memcpy(cmd, payload, length);
    cmd[length] = 0;

    if(!performCommand(cmd)) {
      DLOG("Unknown command %s\n", cmd);
    }
    free(cmd);
  }
}

void debugCallback() {
  String cmd = Debug.getLastCommand();

  // Debugging commands via telnet
  if (performCommand(cmd.c_str())) {
  } else if (cmd == "quit") {
    DLOG("Stopping Roomba\n");
    Serial.write(173);
  } else if (cmd == "rreset") {
    DLOG("Resetting Roomba\n");
    roomba.reset();
  } else if (cmd == "mqtthello") {
    mqttClient.publish("vacuum/hello", "hello there");
  } else if (cmd == "version") {
    const char compile_date[] = __DATE__ " " __TIME__;
    DLOG("Compiled on: %s\n", compile_date);
  } else if (cmd == "baud115200") {
    DLOG("Setting baud to 115200\n");
    roomba.baud(Roomba::Baud115200);
    delay(100);
  } else if (cmd == "baud19200") {
    DLOG("Setting baud to 19200\n");
    roomba.baud(Roomba::Baud19200);
    delay(100);
  } else {
    DLOG("Unknown command %s\n", cmd.c_str());
  }
}

void setup() {
  // High-impedence on the BRC_PIN
  pinMode(BRC_PIN,INPUT);

  // Set Hostname.
  String hostname(HOSTNAME);
  WiFi.hostname(hostname);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();

  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(mqttCallback);

  #if LOGGING
  Debug.begin((const char *)hostname.c_str());
  Debug.setResetCmdEnabled(true);
  Debug.setCallBackProjectCmds(debugCallback);
  Debug.setSerialEnabled(false);
  #endif

  roomba.start();
}

void reconnect() {
  DLOG("Attempting MQTT connection...\n");
  // Attempt to connect
  if (mqttClient.connect(HOSTNAME, MQTT_USER, MQTT_PASSWORD)) {
    DLOG("MQTT connected\n");
    mqttClient.subscribe(commandTopic);
  } else {
    DLOG("MQTT failed rc=%d try again in 5 seconds\n", mqttClient.state());
  }
}

void sendStatus() {
  // Flush serial buffers
  while (Serial.available()) {
    Serial.read();
  }

  uint8_t sensors[] = {
    Roomba::SensorDistance, // 2 bytes, mm, signed
    Roomba::SensorChargingState, // 1 byte
    Roomba::SensorVoltage, // 2 bytes, mV, unsigned
    Roomba::SensorCurrent, // 2 bytes, mA, signed
    Roomba::SensorBatteryCharge, // 2 bytes, mAh, unsigned
    Roomba::SensorBatteryCapacity // 2 bytes, mAh, unsigned
  };
  uint8_t values[11];

  bool success = roomba.getSensorsList(sensors, sizeof(sensors), values, 11);
  if (!success) {
    DLOG("Failed to read sensor values from Roomba\n");
    return;
  }
  int16_t distance = values[0] * 256 + values[1];
  uint8_t chargingState = values[2];
  uint16_t voltage = values[3] * 256 + values[4];
  int16_t current = values[5] * 256 + values[6];
  uint16_t charge = values[7] * 256 + values[8];
  uint16_t capacity = values[9] * 256 + values[10];

  DLOG("Got sensor values Distance:%dmm ChargingState:%d Voltage:%dmV Current:%dmA Charge:%dmAh Capacity:%dmAh\n", distance, chargingState, voltage, current, charge, capacity);

  cleaning = false;
  docked = false;
  if (current < -400) {
    cleaning = true;
  } else if (current > -50) {
    docked = true;
  }

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["battery_level"] = (charge * 100)/capacity;
  root["cleaning"] = cleaning;
  root["docked"] = docked;
  root["charging"] = chargingState == Roomba::ChargeStateReconditioningCharging
  || chargingState == Roomba::ChargeStateFullCharging
  || chargingState == Roomba::ChargeStateTrickleCharging;
  root["voltage"] = voltage;
  root["current"] = current;
  root["charge"] = charge;
  String jsonStr;
  root.printTo(jsonStr);
  mqttClient.publish(statusTopic, jsonStr.c_str());
}

int lastStateMsgTime = 0;
int lastWakeupTime = 0;

void loop() {
  long now = millis();
  // If MQTT client can't connect to broker, then reconnect
  if (!mqttClient.connected()) {
    reconnect();
  } else {
    if (now - lastWakeupTime > 50000) {
      lastWakeupTime = now;
      if (!cleaning) {
        if (docked) {
          wakeOnDock();
        } else {
          wakeOffDock();
        }
      }
    }
    if (now - lastStateMsgTime > 10000) {
      lastStateMsgTime = now;
      sendStatus();
    }
  }

  ArduinoOTA.handle();
  yield();
  Debug.handle();
  mqttClient.loop();
}
