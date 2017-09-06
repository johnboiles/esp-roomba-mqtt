// Roomba.h
//
///
/// \mainpage Roomba library for Arduino
///
/// This is the Arduino Roomba library.
/// It provides an object-oriented interface for talking to iRobot Roomba and Create robots
/// via a serial port.
/// The Roomba is the original device from iRobot. It has a serial port DIN socket on the side.
/// The Create is a more full-featured robot platform for hobbyists. It has both the DIN socket and a DB25
/// connector called the CArgo Bay Connector
///
/// The Create understands a superset of the Roomba commands. This library supports both devices. Where 
/// comands are only available on one or the other platform it is noted.
///
/// The version of the package that this documentation refers to can be downloaded 
/// from http://www.airspayce.com/mikem/arduino/Roomba/Roomba-1.3.zip
/// You can find the latest version at http://www.airspayce.com/mikem/arduino/Roomba
///
/// Tested on Arduino Duemilanove, Diecimila and Mega with arduino-0018 on OpenSuSE 11.1 and avr-libc-1.6.1-1.15,
/// cross-avr-binutils-2.19-9.1, cross-avr-gcc-4.1.3_20080612-26.5.
///
/// A number of example programs are included that work with the iRobot Create:
/// \li TestSuite Runs on a Mega and exercises a number of the functions, checking for correct operation
/// \li RoombaTest1 Runs in any Arduino, checks that sensors can be read from Create using the stream() command
/// \li RoombaRCRx Demo program that shows how to control a Create using the RCRx Arduino library,
/// an Arduino WiFi shield such as BlackWidow and the RCTx 
/// iPhone app. Control a Create from your iPhone!
///
/// A video domonstating this library (along with the http://www.airspayce.com/mikem/arduino/RCKit library)
/// being used to control a Create from an iPhone or iPad can be found at http://www.youtube.com/watch?v=Qv-5ZOb5WW4
///
/// \par Installation
/// Install in the usual way: unzip the distribution zip file to the libraries
/// sub-folder of your sketchbook. 
///
/// This software is Copyright (C) 2010 Mike McCauley. Use is subject to license
/// conditions. The main licensing options available are GPL V2 or Commercial:
/// 
/// \par Open Source Licensing GPL V2
/// This is the appropriate option if you want to share the source code of your
/// application with everyone you distribute it to, and you also want to give them
/// the right to share who uses it. If you wish to use this software under Open
/// Source Licensing, you must contribute all your source code to the open source
/// community in accordance with the GPL Version 2 when your application is
/// distributed. See http://www.gnu.org/copyleft/gpl.html
/// 
/// \par Commercial Licensing
/// This is the appropriate option if you are creating proprietary applications
/// and you are not prepared to distribute and share the source code of your
/// application. Contact info@airspayce.com for details.
///
/// \par Revision History
/// \version 1.0 Initial release
/// \version 1.1 Updated docs, added Youtube video
/// \version 1.2 Compiles under Arduino 1.0
/// \version 1.3  Updated author and distribution location details to airspayce.com
///
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2010 Mike McCauley
// $Id: Roomba.h,v 1.1 2010/09/27 21:58:32 mikem Exp mikem $

#ifndef Roomba_h
#define Roomba_h

#if (ARDUINO < 100)
#include "WProgram.h"
#else
#include <Arduino.h>
#endif

/// Masks for LEDs in leds()
#define ROOMBA_MASK_LED_NONE    0
#define ROOMBA_MASK_LED_PLAY    0x2
#define ROOMBA_MASK_LED_ADVANCE 0x8

/// Masks for digitalOut()
#define ROOMBA_MASK_DIGITAL_OUT_0 0x1
#define ROOMBA_MASK_DIGITAL_OUT_1 0x2
#define ROOMBA_MASK_DIGITAL_OUT_2 0x4

/// Masks for drivers()
#define ROOMBA_MASK_DRIVER_0 0x1
#define ROOMBA_MASK_DRIVER_1 0x2
#define ROOMBA_MASK_DRIVER_2 0x4
/// Roomba only:
#define ROOMBA_MASK_SIDE_BRUSH 0x1
#define ROOMBA_MASK_VACUUM     0x2
#define ROOMBA_MASK_MAIN_BRUSH 0x4

/// Masks for bumps and wheedrops sensor packet id 7
#define ROOMBA_MASK_BUMP_RIGHT       0x1
#define ROOMBA_MASK_BUMP_LEFT        0x2
#define ROOMBA_MASK_WHEELDROP_RIGHT  0x4
#define ROOMBA_MASK_WHEELDROP_LEFT   0x8
#define ROOMBA_MASK_WHEELDROP_CASTER 0x10

/// Masks for driver overcurrents Packet ID 13
#define ROOMBA_MASK_LD1              0x1
#define ROOMBA_MASK_LD0              0x2
#define ROOMBA_MASK_LD2              0x4
#define ROOMBA_MASK_RIGHT_WHEEL      0x8
#define ROOMBA_MASK_LEFT_WHEEL       0x10
// Roomba, use ROOMBA_MASK_SIDE_BRUSH,  ROOMBA_MASK_VACUUM, ROOMBA_MASK_MAIN_BRUSH

/// Masks for buttons sensor packet ID 18
/// Create
#define ROOMBA_MASK_BUTTON_PLAY      0x1
#define ROOMBA_MASK_BUTTON_ADVANCE   0x4
/// Roomba
#define ROOMBA_MASK_BUTTON_MAX       0x1
#define ROOMBA_MASK_BUTTON_CLEAN     0x2
#define ROOMBA_MASK_BUTTON_SPOT      0x4
#define ROOMBA_MASK_BUTTON_POWER     0x8

/// Masks for digital inputs sensor packet ID 32
#define ROOMBA_MASK_DIGITAL_IN_0      0x1
#define ROOMBA_MASK_DIGITAL_IN_1      0x2
#define ROOMBA_MASK_DIGITAL_IN_2      0x4
#define ROOMBA_MASK_DIGITAL_IN_3      0x8
#define ROOMBA_MASK_DIGITAL_IN_DEVICE_DETECT      0x10

/// Masks for charging sources sensor packet ID 34
#define ROOMBA_MASK_INTERNAL_CHARGER  0x1
#define ROOMBA_MASK_HOME_BASE         0x2

/// \def ROOMBA_READ_TIMEOUT
/// Read timeout in milliseconds.
/// If we have to wait more than this to read a char when we are expecting one, then something is wrong.
#define ROOMBA_READ_TIMEOUT 200

// You may be able to set this so you can use Roomba with NewSoftSerial
// instead of HardwareSerial
//#define HardwareSerial NewSoftSerial

/////////////////////////////////////////////////////////////////////
/// \class Roomba Roomba.h <Roomba.h>
/// \brief Support for iRobot Roomba and Create platforms via serial port using the iRobot Open Interface (OI)
/// protocol.
///
/// The iRobot Roomba and Create platforms support a serial port through which you can control and 
/// interrogate the device. The protocol implemented here conforms to the Open Interface protocol described in the 
/// iRobot Open Interface Command Reference. Not all commands are supported on both platforms. Differences are
/// noted in the API
///
/// The Roomba and Create is equipped with a min-din serial port socket, while the Create
/// is also equipped with a 25-pin D connector called the Cargo Bay Connector. The
/// pins on the Cargo Bay Connector include the serial port, battery, digital inputs and
/// outputs, and an analog input.
///
/// In order to communicate with a Roomba, you must create an instance of the Roomba class and then call its 
/// instance methods to send commmands and receive sensor messages. You can also request continuous 
/// streams of sensor data to be sent by the Roomba. The Roomba also emits messages on its 
/// serial port from time to time as described below.
///
/// \par Electrical Considerations
///
/// The serial port output TXD from the Roomba/Create is too weak to drive the RX serial port input 
/// of an Arduino properly.
/// This is because of the USB-Serial converter on the Arduino: it also tries to drive the RX serial port input
/// via a pullup resistor, 
/// but the Roomba does not have enough drive to pull the RX down below about 2.5 volts, which is insufficient 
/// to be reliably detected as a TTL serial input of 0.
///
/// \note Note that this problem does not affect the Serial1 Serial2 or Serial3 ports on the Mega: these ports do not 
/// have other circuitry connected to them and the Roomba can drive the serial port inputs of these ports 
/// without a problem. Its only the RX Serial port (ie the first Serial port) that is affected by this problem. 
///
/// What this means is that if you intend to connect a Roomba to the standard (first) RX/TX serial port on an Arduino 
/// you \a MUST have additional circuitry to help drive RX on the Arduino low enough. 
/// We use a simple PNP emitter follower. Almost any small signal low power PNP will do.
/// See the <a href="Create-Arduino.pdf">example circuit diagram</a>. This will ensure the RX serial 
/// port input on the Arduino is pulled down to about 0.6V for reliable comms.
///
/// \par Other Roomba messages
///
/// When iRobot Create powers up and after a reset, it sends a message like this on its serial port:
/// \code
/// bl-start
/// 2006-09-12-1137-L   
/// RDK by iRobot!
/// MC9S12E128
/// 2006-11-20-1731-L   
/// battery-current-quiescent-raw 524  battery-current-zero 510
/// 
/// 2006-11-20-1731-L   
/// \endcode
///
/// While charging it will send a message like this each second:
/// \code
/// bat:   min 3  sec 21  mV 15558  mA 1491  deg-C 24  
/// \endcode
///
/// To enter the factory test menu for the IRobot Create, hold down the (>) and (>>|) 
/// buttons then press and hold the Power button until the assending and descending tones play and then stop. 
/// You wil see some messages emitted on teh serial port.
/// Press the right-right arrow button to cycle through the tests.
///
class Roomba
{
public:
    /// \enum Baud
    /// Demo types to pass to Roomba::baud()
    typedef enum
    {
	Baud300    = 0,
	Baud600    = 1,
	Baud1200   = 2,
	Baud2400   = 3,
	Baud4800   = 4,
	Baud9600   = 5,
	Baud14400  = 6,
	Baud19200  = 7,
	Baud28800  = 8,
	Baud38400  = 9,
	Baud57600  = 10,
	Baud115200 = 11,
    } Baud;
    
    /// \enum Demo
    /// Demo types to pass to Roomba::demo()
    typedef enum
    {
	DemoAbort = -1,
	DemoCover = 0,
	DemoCoverAndDock = 1,
	DemoSpotCover = 2,
	DemoMouse = 3,
	DemoDriveFigureEight = 4,
	DemoWimp = 5,
	DemoHome = 6,
	DemoTag = 7,
	DemoPachelbel = 8,
	DemoBanjo = 9,
    } Demo;
    
    /// \enum Drive
    /// Special values for radius in Roomba::drive()
    typedef enum
    {
	DriveStraight                = 0x8000,
	DriveInPlaceClockwise        = 0xFFFF,
	DriveInPlaceCounterClockwise = 0x0001,
    } Drive;
  
    /// \enum StreamCommand
    /// Values to pass to Roomba::streamCommand()
    typedef enum
    {
	StreamCommandPause  = 0,  
	StreamCommandResume = 1,
    } StreamCommand;
  
    /// \enum EventType
    /// Values to pass to Roomba::waitEvent()
    typedef enum
    {
	EventTypeWheelDrop       = 1,
	EventTypeFronWheelDrop   = 2,
	EventTypeLeftWheelDrop   = 3,
	EventTypeRightWheelDrop  = 4,
	EventTypeBump            = 5,
	EventTypeLeftBump        = 6,
	EventTypeRightBump       = 7,
	EventTypeVirtualWall     = 8,
	EventTypeWall            = 9,
	EventTypeCliff           = 10,
	EventTypeLeftCliff       = 11,
	EventTypeFrontLeftCliff  = 12,
	EventTypeFrontRightCliff = 13,
	EventTypeRightCliff      = 14,
	EventTypeHomeBase        = 15,
	EventTypeAdvanceButton   = 16,
	EventTypePlayButton      = 17,
	EventTypeDigitalInput0   = 18,
	EventTypeDigitalInput1   = 19,
	EventTypeDigitalInput2   = 20,  
	EventTypeDigitalInput3   = 21,
	EventTypeModePassive     = 22,
    } EventType;
  
    /// \enum IRCommand
    /// Values for sensor packet ID 27
    typedef enum
    {
	// Remote control:
	IRCommandLeft                   = 129,
	IRCommandForward                = 130,
	IRCommandRight                  = 131,
	IRCommandSpot                   = 132,
	IRCommandMax                    = 133,
	IRCommandSmall                  = 134,
	IRCommandMedium                 = 135,
	IRCommandLargeClean             = 136,
	IRCommandPause                  = 137,
	IRCommandPower                  = 138,
	IRCommandArcForwardLeft         = 139,
	IRCommandArcForwardRight        = 140,
	IRCommandDriveStop              = 141,
	// Scheduling Remote:
	IRCommandSendAll                = 142,
	IRCommandSeekDock               = 143,
	// Home Base:
	IRCommandReserved1              = 240,
	IRCommandRedBuoy                = 248,
	IRCommandGreenBuoy              = 244, 
	IRCommandForceField             = 242,
	IRCommandRedGreenBuoy           = 252,
	IRCommandRedBuoyForceField      = 250,
	IRCommandGreenBuoyForceField    = 246,
	IRCommandRedGreenBuoyForceField = 254,
    } IRCommand;
  
    /// \enum ChargeState
    /// Values for sensor packet ID 21
    typedef enum
    {
	ChargeStateNotCharging            = 0,
	ChargeStateReconditioningCharging = 1,
	ChargeStateFullChanrging          = 2,
	ChargeStateTrickleCharging        = 3,
	ChargeStateWaiting                = 4,
	ChargeStateFault                  = 5,
    } ChargeState;
  
    /// \enum Mode
    /// Values for sensor packet ID 35
    typedef enum
    {
	ModeOff     = 0,
	ModePassive = 1,
	ModeSafe    = 2,
	ModeFull    = 3,
    } Mode;
  
    /// \enum Sensor
    /// Values for sensor packet IDs to pass to getSensors() and getSensorsList()
    typedef enum
    {
	Sensors7to26                   = 0,
	Sensors7to16                   = 1,
	Sensors17to20                  = 2,
	Sensors21to26                  = 3,
	Sensors27to34                  = 4,
	Sensors35to42                  = 5,
	Sensors7to42                    = 6,
	SensorBumpsAndWheelDrops       = 7,
	SensorWall                     = 8,
	SensorCliffLeft                = 9,
	SensorCliffFrontLeft           = 10,
	SensorCliffFrontRight          = 11,
	SensorCliffRight               = 12,
	SensorVirtualWall              = 13,
	SensorOvercurrents             = 14,
//	SensorUnused1                  = 15,
//	SensorUnused2                  = 16,
	SensorIRByte                   = 17,
	SensorButtons                  = 18,
	SensorDistance                 = 19,
	SensorAngle                    = 20,
	SensorChargingState            = 21,
	SensorVoltage                  = 22,
	SensorCurrent                  = 23,
	SensorBatteryTemperature       = 24,
	SensorBatteryCharge            = 25,
	SensorBatteryCapacity          = 26,
	SensorWallSignal               = 27,
	SensoCliffLeftSignal           = 28,
	SensoCliffFrontLeftSignal      = 29,
	SensoCliffFrontRightSignal     = 30,
	SensoCliffRightSignal          = 31,
	SensorUserDigitalInputs        = 32,
	SensorUserAnalogInput          = 33,
	SensorChargingSourcesAvailable = 34,
	SensorOIMode                   = 35,
	SensorSongNumber               = 36,
	SensorSongPlaying              = 37,
	SensorNumberOfStreamPackets    = 38,
	SensorVelocity                 = 39,
	SensorRadius                   = 40,
	SensorRightVelocity            = 41,
	SensorLeftVelocity             = 42,
    } Sensor;
  
    /// Constructor. You can have multiple simultaneous Roomba if that makes sense.
    /// \param[in] serial POinter to the HardwareSerial port to use to communicate with the Roomba. 
    /// Defaults to &Serial
    /// \param[in] baud the baud rate to use on the serial port. Defaults to 57600, the default for the Roomba.
    Roomba(HardwareSerial* serial = &Serial, Baud baud = Baud57600);

    /// Resets the Roomba. 
    /// It will emit its startup message
    /// Caution, this may take several seconds to complete
    void reset();

    /// Starts the Open Interface and sets the mode to Passive. 
    /// You must send this before sending any other commands.
    /// Initialises the serial port to the baud rate given in the constructor
    void start();
    
    /// Converts the specified baud code into a baud rate in bits per second
    /// \param[in] baud Baud code, one of Roomba::Baud
    /// \return baud rate in bits per second
    uint32_t baudCodeToBaudRate(Baud baud);

    /// Changes the baud rate
    /// Baud is on of the Roomba::Baud enums
    void baud(Baud baud);

    /// Sets the OI to Safe mode.
    /// In Safe mode, the cliff and wheel drop detectors work to prevent Roomba driving off a cliff
    void safeMode();

    /// Sets the OI to Full mode.
    /// In Full mode, the cliff and wheel drop detectors do not stop the motors: you are responsible
    // for full control of the Roomba
    void fullMode();

    /// Puts a Roomba in sleep mode.
    /// Roomba only, no equivalent for Create.
    void power();

    /// Causes roomba to immediately 
    /// seek the docking station.
    /// No equivalent for Create.
    void dock();

    /// Starts the requirested built-in demo
    /// \param[in] demo The demo number. One of Roomba::Demo
    void demo(Demo demo);

    /// Starts the Cover demo
    /// Changes mode to Passive
    void cover();

    /// Starts the Cover and Dock demo
    /// Changes mode to Passive
    void coverAndDock();

    /// Starts the Spot Cover demo
    /// Changes mode to Passive
    void spot();
	
    /// Starts the Roomba driving with a specified wheel velocity and radius of turn
    /// \param[in] velocity Speed in mm/s (-500 to 500 mm/s)
    /// \param[in] radius Radius of the turn in mm. (-2000 to 2000 mm). 
    /// Any of the special values in enum Roomba::Drive may be used instead of a radius value
    void drive(int16_t velocity, int16_t radius);

    /// Starts the Roomba driving with a specified velocity for each wheel
    /// Create only. No equivalent on Roomba.
    /// \param[in] leftVelocity Left wheel velocity in mm/s (-500 to 500 mm/s)
    /// \param[in] rightVelocity Right wheel velocity in mm/s (-500 to 500 mm/s)
    void driveDirect(int16_t leftVelocity, int16_t rightVelocity);

    /// Controls the LEDs on the Create
    /// \param[in] leds Bitmask specifying which LEDs to activate. ORed combination of ROOMBA_MASK_LED_*
    /// \param[in] powerColour The colour of the Power LED. 0 to 255. 0 = green, 255 = red, 
    /// intermediate values are intermediate colours
    /// \param[in] powerIntensity Power LED intensity. 0 to 255. 0 = off, 255 = full intensity
    void leds(uint8_t leds, uint8_t powerColour, uint8_t powerIntensity);

    /// Sets the digital output pins on the Cargo Bay Connector of the Create
    /// Create only. No equivalent on Roomba.
    /// \param[in] out Mask specifiying which outputs to enable. ORed value ROOMBA_MASK_DIGITAL_OUT_*
    void digitalOut(uint8_t out);

    /// Sets the duty cycle for PWM outputs on the low side drivers. These can be use for PWM driving of
    /// motors, lights etc.
    /// Create only. No equivalent on Roomba.
    /// \param[in] dutyCycle0 Duty cycle for low side driver 0. 0 to 128.
    /// \param[in] dutyCycle1 Duty cycle for low side driver 1. 0 to 128.
    /// \param[in] dutyCycle2 Duty cycle for low side driver 2. 0 to 128.
    void pwmDrivers(uint8_t dutyCycle0, uint8_t dutyCycle1, uint8_t dutyCycle2); 

    /// Sets the low side drivers on or off. On the Romba, these control the 3 motors.
    /// \param[in] out Bitmask of putputs to enable. ORed value ROOMBA_MASK_DRIVER_*
    void drivers(uint8_t out);

    /// Sends the requested byte out of the low side driver 1 (pin 23 on the Cargo Bay Connector).
    /// low side driver 1 can be used to drive an IR transmitter to send commands to other Roombas and Creates. 
    /// Create only. No equivalent on Roomba.
    /// \param[in] data Data byte to transmit
    void sendIR(uint8_t data);

    /// Defines a song which can later be played with playSong()
    /// \param[in] songNumber Song number for this song. 0 to 15
    /// \param[in] notes Array of note/duration pairs. See Open Interface manual for details. 2 bytes per note, 
    /// first byte is the note and the second is the duration
    /// \param[in] len Length of notes array in bytes, so this will be twice the number of notes in the song
    void song(uint8_t songNumber, const uint8_t* notes, int len);

    /// Plays a song that has previously been defined by song()
    /// \param[in] songNumber The song number to play. 0 to 15
    void playSong(uint8_t songNumber);

    /// Requests that a stream of sensor data packets be sent by the Roomba.
    /// See the Open Interface manual for details on the resutting data.
    /// The packets will be sent every 15ms.
    /// You can use pollSensors() to receive sensor data streams.
    /// Create only. No equivalent on Roomba.
    /// See the Open Interface maual for more details and limitations.
    /// \param[in] packetIDs Array specifying sensor packet IDs from Roomba::Sensor to be sent.
    /// \param[in] len Number of IDs in packetIDs
    void stream(const uint8_t* packetIDs, int len);

    /// Pause or resume a stream of sensor data packets previously requested by stream()
    /// Create only. No equivalent on Roomba.
    /// \param[in] command One of Roomba::StreamCommand
    void streamCommand(StreamCommand command);

    /// Defines a command script which can later be executed with playScript(). You can clear the script by calling 
    /// script(NULL, 0);
    /// Create only. No equivalent on Roomba.
    /// \param[in] script Array containing a sequence of Roomba OI commands.
    /// \param[in] len Length of the script in bytes.
    void script(const uint8_t* script, uint8_t len);

    /// Executes a previously defined script, 
    /// the last one specified by script()
    /// Create only. No equivalent on Roomba.
    void playScript();

    /// Tells the Roomba to wait for a specified time.
    /// This command is intended for use in scripting only.
    /// Create only. No equivalent on Roomba.
    /// \param[in] ticks The number of ticks to wait. Each tick is 15ms
    void wait(uint8_t ticks);

    /// Causes Roomba to wait until it has travelled the distance specified. 
    /// Roomba will not react to any inputs until the wait has completed. 
    /// Note that this does not cause the host arduino to wait, it only sends the wait comman to the Roomba
    /// This command is intended for use in scripting only.
    /// Create only. No equivalent on Roomba.
    /// \param[in] mm Distance to wait for in mm
    void waitDistance(int16_t mm);

    /// Causes Roomba to wait until it has rotated through the specified angle.
    /// Roomba will not react to any inputs until the wait has completed. 
    /// Note that this does not cause the host arduino to wait, it only sends the wait comman to the Roomba
    /// This command is intended for use in scripting only.
    /// Create only. No equivalent on Roomba.
    /// \param[in] degrees Angle to wait for in degrees
    void waitAngle(int16_t degrees);

    /// Cause the Create to wait for a specified event.
    /// Roomba will not react to any inputs until the wait has completed. 
    /// Note that this does not cause the host arduino to wait, it only sends the wait comman to the Roomba
    /// Create only. No equivalent on Roomba.
    /// \param[in] type Event type to wait for. One of Roomba::EventType
    void waitEvent(EventType type);

    /// Low level funciton to read len bytes of data from the Roomba
    /// Blocks untill all len bytes are read or a read timeout occurs.
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len Number of bytes to read
    /// \return true if all len bytes were successfully read. Returns false in the case of a timeout 
    /// on reading any byte.
    bool getData(uint8_t* dest, uint8_t len);

    /// Reads the sensor data for the specified sensor packet ID. Note that different sensor packets have 
    /// different lengths, and it is the callers responsibilty to make sure len agrees with the expected 
    /// length of the sensor data. See the Open Interface manual for details on sensor packet lengths.
    /// Roomba.h defines various enums and defines for decoding sensor data.
    /// Blocks untill all len bytes are read or a read timeout occurs.
    /// \param[in] packetID The ID of the sensor packet to read from Roomba::Sensor
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len Number of sensor data bytes to read
    /// \return true if all len bytes were successfully read. Returns false in the case of a timeout 
    /// on reading any byte.
    bool getSensors(uint8_t packetID, uint8_t* dest, uint8_t len);

    /// Reads the sensor data for the specified set of sensor packet IDs. Note that different sensor packets have 
    /// different lengths, and it is the callers responsibilty to make sure len agrees with the expected 
    /// length of the sensor data. See the Open Interface manual for details on sensor packet lengths.
    /// Blocks until all len bytes are read or a read timeout occurs.
    /// Create only. No equivalent on Roomba.
    /// \param[in] packetIDs Array of IDs  from Roomba::Sensor of the sensor packets to read
    /// \param[in] numPacketIDs number of IDs in the packetIDs array
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len Number of sensor data bytes to read and store to dest.
    /// \return true if all len bytes were successfully read. Returns false in the case of a timeout 
    /// on reading any byte.
    bool getSensorsList(uint8_t* packetIDs, uint8_t numPacketIDs, uint8_t* dest, uint8_t len);

    /// Polls the serial input for data belonging to a sensor data stream previously requested with stream().
    /// As sensor data is read it is appended to dest until at most len bytes are stored there. 
    /// When a complete sensor stream has been read with a correct checksum, returns true. 
    /// See the Open Interface manual for details on how the sensor data will be encoded in dest.
    /// Discards characters that are not part of a stream, such as the messages the Roomba 
    /// sends at startup and while charging.
    /// Create only. No equivalent on Roomba.
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len Max number of sensor data bytes to store to dest
    /// \return true when a complete stream has been read, and the checksum is correct. The sensor data
    /// (at most len bytes) will have been stored into dest, ready for the caller to decode.
    bool pollSensors(uint8_t* dest, uint8_t len);

    /// Reads a the contents of the script most recently specified by a call to script().
    /// Create only. No equivalent on Roomba.
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len The maximum number of bytes to place in dest. If the script is actually longer than len
    /// only len bytes will be written
    /// \return The actual number of bytes in the script, even if this is more than len. By calling 
    /// getScript(NULL, 0), you can determine how many bytes would be required to store the script.
    uint8_t getScript(uint8_t* dest, uint8_t len);
  
private:
    /// \enum PollState
    /// Values for _pollState
    typedef enum
    {
	PollStateIdle         = 0,
	PollStateWaitCount    = 1,
	PollStateWaitBytes    = 2,
	PollStateWaitChecksum = 3,
    } PollState;

    /// The baud rate to use for the serial port
    uint32_t        _baud;
	
    /// The serial port to use to talk to the Roomba
    HardwareSerial* _serial;
    
    /// Variables for keeping track of polling of data streams
    uint8_t         _pollState; /// Current state of polling, one of Roomba::PollState
    uint8_t         _pollSize;  /// Expected size of the data stream in bytes
    uint8_t         _pollCount; /// Num of bytes read so far
    uint8_t         _pollChecksum; /// Running checksum counter of data bytes + count

};

#endif
