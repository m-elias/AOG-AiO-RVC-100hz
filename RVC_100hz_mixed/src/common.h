#include "HardwareSerial.h"
#include "elapsedMillis.h"
#include <stdint.h>
#include <Streaming.h>
#include "zADS1115.h"
#include <AsyncWebServer_Teensy41.h>

AsyncWebServer server(80); // Start web server OTA updates.

bool ota_apply = 0;

#include "FXUtil.h" // read_ascii_line(), hex file support
extern "C"
{
#include "FlashTxx.h" // TLC/T3x/T4x/TMM flash primitives
}

#include "LEDS.h"
LEDS LEDs = LEDS(1000, 255, 64, 127); // 1000ms RGB update, 255/64/127 RGB brightness balance levels for v5.0a

ProcessorUsage BNOusage((char *)"BNO   ");
ProcessorUsage GPS1usage((char *)"GPS1  ");
ProcessorUsage GPS2usage((char *)"GPS2  ");
ProcessorUsage PGNusage((char *)"PGN   ");
ProcessorUsage ASusage((char *)"AG    ");
ProcessorUsage NTRIPusage((char *)"NTRIP ");
ProcessorUsage RS232usage((char *)"RS232 ");
ProcessorUsage LOOPusage((char *)"Loop  ");
ProcessorUsage IMU_Husage((char *)"IMU_H ");
ProcessorUsage NMEA_Pusage((char *)"NMEA_H");
ProcessorUsage RTKusage((char *)"Radio ");
ProcessorUsage UBX_Pusage((char *)"UBX_H ");
ProcessorUsage UDP_Susage((char *)"UDP_S ");
ProcessorUsage DACusage((char *)"DAC   ");
ProcessorUsage MACHusage((char *)"MACH  ");
ProcessorUsage LEDSusage((char *)"LEDS  ");
ProcessorUsage ESP32usage((char *)"ESP32 ");
const uint8_t numCpuUsageTasks = 17;
ProcessorUsage *cpuUsageArray[numCpuUsageTasks] = {
    &BNOusage, &GPS1usage, &GPS2usage, &PGNusage, &ASusage, &NTRIPusage,
    &RS232usage, &LOOPusage, &IMU_Husage, &NMEA_Pusage, &RTKusage, &UBX_Pusage,
    &UDP_Susage, &DACusage, &MACHusage, &LEDSusage, &ESP32usage};
HighLowHzStats gps2Stats;
HighLowHzStats gps1Stats;
HighLowHzStats relJitterStats;
HighLowHzStats relTtrStats;
HighLowHzStats bnoStats;

elapsedMillis bufferStatsTimer;
uint32_t testCounter;
bool printCpuUsages = false;
bool printStats = false;
uint16_t ggaMissed;

// ESP32
bool ESP32Debug = false;
bool ESP32gotLFCR = false;
uint8_t ESP32incomingBytes[254];
uint8_t ESP32incomingBytesLength = 0;

// Autosteer variables
bool autoSteerEnabled = false;
float steerAngleActual = 0;
int16_t steeringPosition = 0; // from steering sensor (WAS)
uint8_t switchByte = 0;
float gpsSpeed;
// On Off
uint8_t guidanceStatus = 0, prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;
float steerAngleSetPoint = 0;
uint8_t steerState = 0;
elapsedMicros aogGpsToAutoSteerLoopTimer;
uint8_t xte = 0;
int16_t pwmDisplay = 0;
int16_t pulseCount = 0; // Steering Wheel Encoder
float sensorReading;
uint8_t aog2Count = 0;
const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
float highLowPerDeg = 0;
const float LOW_HIGH_DEGREES = 3.0; // How many degrees before decreasing Max PWM
uint8_t prevSteerReading = 1;
// Variables for settings - 0 is false
struct SteerConfigStruct
{
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0; // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0; // 1 if switch selected
  uint8_t SteerButton = 0; // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 3;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0; // Set to 0 to use X Axis, 1 to use Y avis
  uint8_t MinSpeed = 0;
};
SteerConfigStruct const defaultSteerConfig; // 9 bytes
struct SteerConfigStruct steerConfig = defaultSteerConfig;

// Variables for settings
struct SteerSettingsStruct
{
  uint8_t Kp = 40;     // proportional gain
  uint8_t lowPWM = 10; // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 150; // max PWM value
  float steerSensorCounts = 120;
  float AckermanFix = 1; // sent as percent
};
SteerSettingsStruct defaultSteerSettings;                        // 11 bytes
struct SteerSettingsStruct steerSettings = defaultSteerSettings; // don't need 'struct' in front?

bool aogGpsToAutoSteerLoopTimerEnabled;

// #include "reset.h"    // no on board buttons for reset
// const uint8_t RESET_BTN = A14;          // A13 on Matt's v4.0 test board, A14 ununsed on v5.0a
// RESET teensyReset(RESET_BTN, 2000, LED_BUILTIN);           // reset.h - btn IO, factory reset PERIOD (ms), led IO

#include "Encoder.h"
Encoder encoder(KICKOUT_D_PIN, KICKOUT_A_PIN); // read single or double input values in Autosteer.ino

#include <ADC.h>
#include <ADC_util.h>
ADC *teensyADC = new ADC();                    // 16x oversampling medium speed 12 bit A/D object (set in Autosteer.ino)
ADS1115_lite ads1115(ADS1115_DEFAULT_ADDRESS); // Use this for the 16-bit version ADS1115
// moved to HWv50a const int16_t WorkSWAnanlogThres = 4055;// 100;
const uint8_t ANALOG_TRIG_HYST = 10;

#include "BNO_RVC.h"
BNO_RVC BNO; // Roomba Vac mode for BNO085

#include "QNEthernet.h"
using namespace qindesign::network;
#include "clsPCA9555.h" // https://github.com/nicoverduin/PCA9555
PCA9555 outputs(0x20);  // 0x20 - I2C addr (A0-A2 grounded), interrupt pin causes boot loop

#include "machine.h"
bool SConAiO_InUse = false;
MACHINE machine;                                                   // also used for v4 as it suppressing machine PGN debug messages
const uint8_t pcaOutputPinNumbers[8] = {1, 0, 12, 15, 9, 8, 6, 7}; // all 8 PCA9555 section/machine output pin numbers on v5.0a
const uint8_t pcaInputPinNumbers[] = {14, 13, 11, 10, 2, 3, 4, 5}; // all 8 PCA9555 section/machine output "sensing" pin numbers on v5.0a

#include "Eth_UDP.h"
Eth_UDP UDP;

#include "zNMEA.h"
NMEAParser<4> nmeaParser; // A parser is declared with 3 handlers at most
bool nmeaDebug = false, extraCRLF;

#include "zUBXParser.h"
UBX_Parser ubxParser;

#include "zFUSEImu.h"
FUSE_Imu fuseImu;

// Keya CAN bus
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> Keya_Bus;
int8_t KeyaCurrentSensorReading = 0;
bool keyaDetected = false;

#define PANDA_SINGLE 1
#define PAOGI_DUAL 0
bool startup = false;
// elapsedMillis gpsLostTimer;
elapsedMillis LEDTimer;
elapsedMillis imuPandaSyncTimer;
bool ggaReady, imuPandaSyncTrigger;
bool ggaTimeout, relposnedTimeout;
bool SerialGPSactive, SerialGPS2active;
uint32_t dualTime;

constexpr int buffer_size = 256;
uint8_t GPSrxbuffer[buffer_size];  // Extra GPS1 serial rx buffer
uint8_t GPStxbuffer[buffer_size];  // Extra GPS1 serial tx buffer
uint8_t GPS2rxbuffer[buffer_size]; // Extra GPS2 serial rx buffer
uint8_t GPS2txbuffer[buffer_size]; // Extra GPS2 serial tx buffer
uint8_t RTKrxbuffer[buffer_size];  // Extra RTK serial rx buffer
#ifdef AIOv50a
uint8_t RS232txbuffer[buffer_size]; // Extra RS232 serial tx buffer
// uint8_t RS232rxbuffer[buffer_size];   // Extra RS232 serial rx buffer (not needed unless custom rs232 rx code is added)
uint8_t ESP32rxbuffer[buffer_size]; // Extra ESP32 serial rx buffer
uint8_t ESP32txbuffer[buffer_size]; // Extra ESP32 serial tx buffer
#endif

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype to set CPU speed

// UDP Passthrough
bool udpPassthrough = false; // False = GPS neeeds to send GGA, VTG & HPR messages. True = GPS needs to send KSXT messages only.
bool gotCR = false;
bool gotLF = false;
bool gotDollar = false;
byte gps1MsgBuf[254];
int gps1MsgBufLen = 0;
