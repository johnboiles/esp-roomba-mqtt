#include "secrets.h"

#define HOSTNAME "roomba" // e.g. roomba.local
#define BRC_PIN 14
#define ROOMBA_650_SLEEP_FIX 1
#ifndef ROOMBA_650_SLEEP_FIX
#define ROOMBA_500 1
#endif
#define SET_DATETIME 1
#define TIMEZONE 3

#define ADC_VOLTAGE_DIVIDER 44.551316985
//#define ENABLE_ADC_SLEEP

#define MQTT_SERVER "10.0.0.2"
#define MQTT_USER "homeassistant"
#define MQTT_COMMAND_TOPIC "vacuum/command"
#define MQTT_STATE_TOPIC "vacuum/state"