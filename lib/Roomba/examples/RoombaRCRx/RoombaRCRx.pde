// RCRx
//
// Sample RCRx RCOIP receiver for driving a Create
// Receives RCOIP commmands on a WiShield and uses them to control the wheel motors on a Create
// Uses the driveDirect command so cant be used on Roomba
//
// This compiles to about 15000 bytes on Duemilanove, and will not fit in Diecimila.
//
// This simple example handles 2 RCOIP receiver channels.
//
// This can be used off the shelf with the RCTx transmitter app for iPhone
// Copyright (C) 2010 Mike McCauley

#include <WiShield.h>
#include <RCRx.h>
#include <Servo.h>
//#include "AnalogSetter.h"
//#include "HBridgeSetter.h"
#include "DifferentialSetter.h"
#include "DigitalSetter.h"
#include "AccelStepper.h"
#include "SetterDebug.h"
#include "Roomba.h"

// We handle 2 channels:

// Declare the receiver object
RCRx rcrx;

// Declare the Roomba controller on the default port Serial
Roomba roomba(&Serial);

// We use SetterDebug so we can extract the output values from them during the polling loop
SetterDebug left;
SetterDebug right;
DifferentialSetter ds1(&right, &left);
DifferentialLRSetter di1(&ds1);

// This array of all the outputs is in channel-number-order so RCRx knows which
// Setter to call for each channel received. We define an array of 2 Setters for receiver channels 0 through 1
#define NUM_OUTPUTS 2
Setter*  outputs[NUM_OUTPUTS] = {&di1, &ds1};

void setup()
{
  //  Serial.begin(9600);

  // Ensure the default value of the wheels is stopped
  left.input(127);
  right.input(127);
  
  // Tell the receiver where to send the 2 channels
  rcrx.setOutputs((Setter**)&outputs, NUM_OUTPUTS);
  
  // Initialise the receiver
  rcrx.init();
}

unsigned long lastPollTime = 0;
bool connected = false;

void loop()
{
  // Do WiFi processing
  rcrx.run();
  
  // See if its time to poll the Roomba
  unsigned long time = millis();
  if (time > lastPollTime + 50)
  {
    lastPollTime = time;
    
    uint8_t buf[52]; 
    bool ret;    
    if (!connected)
    {
        roomba.start();
        roomba.fullMode();
        roomba.drive(0, 0); // Stop
        Serial.flush();
        // Try to read a sensor. If that succeeds, we are connected
        ret = roomba.getSensors(Roomba::SensorVoltage, buf, 2);
        connected = ret;
    }
    else
    {
      // Connected
      // Read all sensor data
      // This can take up to 25 ms to complete
      ret = roomba.getSensors(Roomba::Sensors7to42, buf, 52);
      if (ret)
      {
        // got sensor data OK, so still connected
        int32_t leftWheel, rightWheel;

        // Could put some sensor logic here: stop the motors if 
        // we get a bump or  cliff? Go to home base if teh battery voltage is low?
        // etc.
        
        // Compute the output wheel values from the left and right wheel values
        // received from RCRx
        leftWheel = left.lastValue();
        leftWheel = ((-leftWheel + 127) * 500) / 127;
        rightWheel = right.lastValue();
        rightWheel = ((-rightWheel + 127) * 500) / 127;
        // Make close to stopped into stopped
        if (leftWheel < 20 && leftWheel > -20)
          leftWheel = 0;
        if (rightWheel < 20 && rightWheel > -20)
          rightWheel = 0;
        // Send the new values to the roomba
        roomba.driveDirect(leftWheel, rightWheel);
      }
      else
        connected = false;
    }
  }
}


