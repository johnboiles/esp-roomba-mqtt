// Roomba.cpp
//
// Copyright (C) 2010 Mike McCauley
// $Id: Roomba.cpp,v 1.1 2010/09/27 21:58:32 mikem Exp mikem $

#include "Roomba.h"

Roomba::Roomba(HardwareSerial* serial, Baud baud)
{
  _serial = serial;
  _baud = baudCodeToBaudRate(baud);
  _pollState = PollStateIdle;
}

// Resets the
void Roomba::reset()
{
    _serial->write(7);
}

// Start OI
// Changes mode to passive
void Roomba::start()
{
    _serial->begin(_baud);
    _serial->write(128);
}

uint32_t Roomba::baudCodeToBaudRate(Baud baud)
{
    switch (baud)
    {
	case Baud300:
	    return 300;
	case Baud600:
	    return 600;
	case Baud1200:
	    return 1200;
	case Baud2400:
	    return 2400;
	case Baud4800:
	    return 4800;
	case Baud9600:
	    return 9600;
	case Baud14400:
	    return 14400;
	case Baud19200:
	    return 19200;
	case Baud28800:
	    return 28800;
	case Baud38400:
	    return 38400;
	case Baud57600:
	    return 57600;
	case Baud115200:
	    return 115200;
	default:
	    return 57600;
    }
}

void Roomba::baud(Baud baud)
{
    _serial->write(129);
    _serial->write(baud);

    _baud = baudCodeToBaudRate(baud);
    _serial->begin(_baud);
}

void Roomba::safeMode()
{
  _serial->write(131);
}

void Roomba::fullMode()
{
  _serial->write(132);
}

void Roomba::power()
{
  _serial->write(133);
}

void Roomba::dock()
{
  _serial->write(143);
}

void Roomba::demo(Demo demo)
{
  _serial->write(136);
  _serial->write(demo);
}

void Roomba::cover()
{
  _serial->write(135);
}

void Roomba::coverAndDock()
{
  _serial->write(143);
}

void Roomba::spot()
{
  _serial->write(134);
}

void Roomba::drive(int16_t velocity, int16_t radius)
{
  _serial->write(137);
  _serial->write((velocity & 0xff00) >> 8);
  _serial->write(velocity & 0xff);
  _serial->write((radius & 0xff00) >> 8);
  _serial->write(radius & 0xff);
}

void Roomba::driveDirect(int16_t leftVelocity, int16_t rightVelocity)
{
  _serial->write(145);
  _serial->write((rightVelocity & 0xff00) >> 8);
  _serial->write(rightVelocity & 0xff);
  _serial->write((leftVelocity & 0xff00) >> 8);
  _serial->write(leftVelocity & 0xff);
}

void Roomba::leds(uint8_t leds, uint8_t powerColour, uint8_t powerIntensity)
{
  _serial->write(139);
  _serial->write(leds);
  _serial->write(powerColour);
  _serial->write(powerIntensity);
}

void Roomba::digitalOut(uint8_t out)
{
  _serial->write(147);
  _serial->write(out);
}

// Sets PWM duty cycles on low side drivers
void Roomba::pwmDrivers(uint8_t dutyCycle0, uint8_t dutyCycle1, uint8_t dutyCycle2)
{
  _serial->write(144);
  _serial->write(dutyCycle2);
  _serial->write(dutyCycle1);
  _serial->write(dutyCycle0);
}

// Sets low side driver outputs on or off
void Roomba::drivers(uint8_t out)
{
  _serial->write(138);
  _serial->write(out);
}

// Modulates low side driver 1 (pin 23 on Cargo Bay Connector)
// with the given IR command
void Roomba::sendIR(uint8_t data)
{
  _serial->write(151);
  _serial->write(data);
}

// Define a song
// Data is 2 bytes per note
void Roomba::song(uint8_t songNumber, const uint8_t* data, int len)
{
    _serial->write(140);
    _serial->write(songNumber);
    _serial->write(len >> 1); // 2 bytes per note
    _serial->write(data, len);
}

void Roomba::playSong(uint8_t songNumber)
{
  _serial->write(141);
  _serial->write(songNumber);
}

// Start a stream of sensor data with the specified packet IDs in it
void Roomba::stream(const uint8_t* packetIDs, int len)
{
  _serial->write(148);
  _serial->write(packetIDs, len);
}

// One of StreamCommand*
void Roomba::streamCommand(StreamCommand command)
{
  _serial->write(150);
  _serial->write(command);
}

// Use len=0 to clear the script
void Roomba::script(const uint8_t* script, uint8_t len)
{
  _serial->write(152);
  _serial->write(len);
  _serial->write(script, len);
}

void Roomba::playScript()
{
  _serial->write(153);
}

// Each tick is 15ms
void Roomba::wait(uint8_t ticks)
{
  _serial->write(155);
  _serial->write(ticks);
}

void Roomba::waitDistance(int16_t mm)
{
  _serial->write(156);
  _serial->write((mm & 0xff00) >> 8);
  _serial->write(mm & 0xff);
}

void Roomba::waitAngle(int16_t degrees)
{
  _serial->write(157);
  _serial->write((degrees & 0xff00) >> 8);
  _serial->write(degrees & 0xff);
}

// Can use the negative of an event type to wait for the inverse of an event
void Roomba::waitEvent(EventType type)
{
  _serial->write(158);
  _serial->write(type);
}

// Reads at most len bytes and stores them to dest
// If successful, returns true.
// If there is a timeout, returns false
// Blocks until all bytes are read
// Caller must ensure there is sufficient space in dest
bool Roomba::getData(uint8_t* dest, uint8_t len)
{
  while (len-- > 0)
  {
    unsigned long startTime = millis();
    while (!_serial->available())
    {
      // Look for a timeout
      if (millis() > startTime + ROOMBA_READ_TIMEOUT)
        return false; // Timed out
    }
    *dest++ = _serial->read();
  }
  return true;
}

bool Roomba::getSensors(uint8_t packetID, uint8_t* dest, uint8_t len)
{
  _serial->write(142);
  _serial->write(packetID);
  return getData(dest, len);
}

bool Roomba::getSensorsList(uint8_t* packetIDs, uint8_t numPacketIDs, uint8_t* dest, uint8_t len)
{
  _serial->write(149);
  _serial->write(numPacketIDs);
  _serial->write(packetIDs, numPacketIDs);
  return getData(dest, len);
}

// Simple state machine to read sensor data and discard everything else
bool Roomba::pollSensors(uint8_t* dest, uint8_t len)
{
    while (_serial->available())
    {
	uint8_t ch = _serial->read();
	switch (_pollState)
	{
	    case PollStateIdle:
		if (ch == 19)
		    _pollState = PollStateWaitCount;
		break;

	    case PollStateWaitCount:
		_pollChecksum = _pollSize = ch;
		_pollCount = 0;
		_pollState = PollStateWaitBytes;
		break;

	    case PollStateWaitBytes:
		_pollChecksum += ch;
		if (_pollCount < len)
		    dest[_pollCount] = ch;
		if (_pollCount++ >= _pollSize)
		    _pollState = PollStateWaitChecksum;
		break;

	    case PollStateWaitChecksum:
		_pollChecksum += ch;
		_pollState = PollStateIdle;
		return (_pollChecksum == 0);
		break;
	}
    }
    return false;
}

// Returns the number of bytes in the script, or 0 on errors
// Only saves at most len bytes to dest
// Calling with len = 0 will return the amount of space required without actually storing anything
uint8_t Roomba::getScript(uint8_t* dest, uint8_t len)
{
  _serial->write(154);

  unsigned long startTime = millis();
  while (!_serial->available())
  {
    // Look for a timeout
    if (millis() > startTime + ROOMBA_READ_TIMEOUT)
      return 0; // Timed out
  }

  int count = _serial->read();
  if (count > 100 || count < 0)
    return 0; // Something wrong. Cant have such big scripts!!

  // Get all the data, saving as much as we can
  uint8_t i;
  for (i = 0; i < count; i++)
  {
    startTime = millis();
    while (!_serial->available())
    {
      // Look for a timeout
      if (millis() > startTime + ROOMBA_READ_TIMEOUT)
        return 0; // Timed out
    }
    uint8_t data = _serial->read();
    if (i < len)
      *dest++ = data;
  }

  return count;
}
