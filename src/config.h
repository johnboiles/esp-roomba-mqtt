#include "secrets.h"

#define HOSTNAME "roomba" // e.g. roomba.local
#define BRC_PIN 14
#define ROOMBA_650_SLEEP_FIX 1

#define ADC_VOLTAGE_DIVIDER 44.551316985
//#define ENABLE_ADC_SLEEP

// to be adjusted by user
#define MQTT_SERVER "10.0.0.2"
#define MQTT_USER "roomba"

// Only change if you know what you're doing!
#define MQTT_DISCOVERY "homeassistant"
#define MQTT_DEVICE_CLASS "vacuum"
#define MQTT_DIVIDER "/"
#define MQTT_TOPIC_BASE MQTT_DISCOVERY MQTT_DIVIDER MQTT_DEVICE_CLASS MQTT_DIVIDER
#define MQTT_IDPREFIX "roomba_"
#define MQTT_COMMAND_TOPIC "command"
#define MQTT_STATE_TOPIC "state"
#define MQTT_CONFIG_TOPIC "config"
