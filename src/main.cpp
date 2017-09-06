#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <Roomba.h>
#include <PubSubClient.h>
#include "config.h"

// Remote debugging over telnet. Just run:
// telnet roomba.local
#if LOGGING
#include <RemoteDebug.h>
#define DLOG(msg, ...) if(Debug.isActive(Debug.DEBUG)){Debug.printf(msg, ##__VA_ARGS__);}
RemoteDebug Debug;
#else
#define DLOG(msg, ...)
#endif

Roomba roomba(&Serial, Roomba::Baud115200);

// Network setup
WiFiClient wifiClient;

PubSubClient mqttClient(wifiClient);
const PROGMEM char *commandTopic = "vacuum/command";

const char compile_date[] = __DATE__ " " __TIME__;

void wakeup(){
  digitalWrite(BRCPIN, LOW);
  delay(500);
  digitalWrite(BRCPIN, HIGH);
 }

bool performCommand(const char *cmdchar) {
  // TODO: do this only if necessary
  wakeup();
  roomba.start();

  String cmd(cmdchar);

  // MQTT protocol commands
  if (cmd == "turn_on") {
    DLOG("Turning on\n");
    roomba.cover();
  } else if (cmd == "turn_off") {
    DLOG("Turning off\n");
    roomba.power();
  } else if (cmd == "toggle") {
    DLOG("Toggling\n");
    roomba.cover();
  } else if (cmd == "stop") {
    DLOG("Stopping\n");
    roomba.cover();
  } else if (cmd == "clean_spot") {
    DLOG("Cleaning Spot\n");
    roomba.spot();
  } else if (cmd == "locate") {
    DLOG("Locating\n");
    // TODO
  } else if (cmd == "return_to_base") {
    DLOG("Returning to Base\n");
    roomba.dock();
  } else {
    return false;
  }
  return true;
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  DLOG("Received mqtt callback for topic %s\n", topic);
  // If the 'garage/button' topic has a payload "OPEN", then trigger the relay
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

  // MQTT protocol commands
  if (performCommand(cmd.c_str())) {
  }
  // Debugging commands
  else if (cmd == "sensors") {
    DLOG("Querying Sensors\n");
    Serial.write(128);
    delay(50);
    Serial.write(142);
    delay(50);
    Serial.write(100);
  } else if (cmd == "quit") {
    DLOG("Stopping Roomba\n");
    Serial.write(173);
  } else if (cmd == "rreset") {
    DLOG("Resetting Roomba\n");
    roomba.reset();
  } else if (cmd == "safe") {
    DLOG("Changing Roomba to Safe mode\n");
    Serial.write(128);
    // delay(50);
    Serial.write(131);
    // delay(50);
  } else if (cmd == "full") {
    DLOG("Changing Roomba to full mode\n");
    Serial.write(128);
    // delay(50);
    Serial.write(132);
    // delay(50);
  } else if (cmd == "sensor") {
    DLOG("Querying Sensors\n");
    Serial.write(128);
    delay(50);
    Serial.write(142);
    delay(50);
    Serial.write(7);
  } else if (cmd == "mode") {
    DLOG("Querying Mode\n");
    Serial.write(128);
    delay(50);
    Serial.write(142);
    delay(50);
    Serial.write(35);
  } else if (cmd == "battery") {
    DLOG("Querying Battery\n");
    Serial.write(128);
    delay(50);
    Serial.write(142);
    delay(50);
    Serial.write(25);
  } else if (cmd == "capacity") {
    DLOG("Querying Battery Capacity\n");
    Serial.write(128);
    delay(50);
    Serial.write(142);
    delay(50);
    Serial.write(26);
  } else if (cmd == "mqtthello") {
    mqttClient.publish("vacuum/hello", "hello there");
  } else {
    DLOG("Unknown command %s\n", cmd.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  // Serial.println("Built on");
  // Serial.println(compile_date);
  // Serial.println();
  // Serial.print("Connecting to ");
  // Serial.println(ssid);

  pinMode(BRCPIN, OUTPUT);
  digitalWrite(BRCPIN, HIGH);

  // Set Hostname.
  String hostname(HOSTNAME);
  WiFi.hostname(hostname);
  WiFi.begin(ssid, wifiPassword);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }

  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();

  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(mqttCallback);

  #if LOGGING
  Debug.begin((const char *)hostname.c_str());
  Debug.setResetCmdEnabled(true);
  Debug.setCallBackProjectCmds(debugCallback);
  // Debug.setSerialEnabled(false);
  #endif

  // Serial.println("");
  // Serial.println("WiFi connected");
  // Serial.println("IP address: ");
  // Serial.println(WiFi.localIP());
}

void reconnect() {
  DLOG("Attempting MQTT connection...\n");
  // Attempt to connect
  if (mqttClient.connect(HOSTNAME, mqttUser, mqttPassword)) {
    DLOG("MQTT connected\n");
    mqttClient.subscribe(commandTopic);
  } else {
    DLOG("MQTT failed rc=%d try again in 5 seconds\n", mqttClient.state());
  }
}

int lastStateMsgTime = 0;

void loop() {
  long now = millis();
  // If MQTT client can't connect to broker, then reconnect
  if (!mqttClient.connected()) {
    reconnect();
  }

  if (now - lastStateMsgTime > 5000) {
    lastStateMsgTime = now;
    DLOG("Performing periodic publish for state %d\n", mqttClient.state());
  }

  int rx = Serial.read();
  if (rx != -1) {
    Debug.printf("%d\n", (char *)rx);
  }
  ArduinoOTA.handle();
  yield();
  Debug.handle();
  mqttClient.loop();
}
