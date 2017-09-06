// RoombaTest1.pde
//
// Test for Roomba class
// Tests reading of sensor data from a Create using the stream() command
// Starts a stream of sensor data which is read by pollSensors()
//
// Runs on any Arduino with Create connected to Serial
// Tested on Diecimila, Mega and Duemilanove using the 
// RXD driver transistor as described in accompanying docs.
// Turns on the LED when successfully reading sensor data
//
// Copyright (C) 2010 Mike McCauley
// $Id: $

#include <Roomba.h>

// Defines the Roomba instance and the HardwareSerial it connected to
Roomba roomba(&Serial);
int ledPin =  13;

void setup()
{
//    Serial.begin(9600);
    
    pinMode(ledPin, OUTPUT);  
    roomba.start();
    
    // Request a stream of sensor packets
    // List of packet IDs we want included in the stream
    uint8_t packetids[] = { 7, 8, 9 };
    roomba.stream(packetids, sizeof(packetids));
}

void loop()
{
  // Poll for sensor data
  // When a complete set of sensort data is received, returns true
  uint8_t buf[52];
  bool ret = roomba.pollSensors(buf, 52);
  // Turn the LED on if the read was OK, meaning
  // we got a complete sensor stream with a correct checksum and therefore buf is valid
  // and filled with sendor data for the sensor IDs requested in stream()
  if (ret)
    digitalWrite(ledPin, HIGH);
  else
    digitalWrite(ledPin, LOW);
  delay(10);
}
