#include "HardwareSerial.h"
#include "elapsedMillis.h"
#include <stdint.h>
#include <Streaming.h>
#include "zADS1115.h"

#include "LEDS.h"
LEDS LEDs = LEDS(1000, 255, 64, 127);   // 1000ms RGB update, 255/64/127 RGB brightness balance levels for v5.0a

ProcessorUsage BNOusage           ((char*)"BNO   ");
ProcessorUsage GPS1usage          ((char*)"GPS1  ");
ProcessorUsage GPS2usage          ((char*)"GPS2  ");
ProcessorUsage PGNusage           ((char*)"PGN   ");
ProcessorUsage ASusage            ((char*)"AG    ");
ProcessorUsage NTRIPusage         ((char*)"NTRIP ");
ProcessorUsage RS232usage         ((char*)"RS232 ");  
ProcessorUsage LOOPusage          ((char*)"Loop  ");
ProcessorUsage IMU_Husage         ((char*)"IMU_H ");
ProcessorUsage NMEA_Pusage        ((char*)"NMEA_H");
ProcessorUsage RTKusage           ((char*)"Radio ");
ProcessorUsage UBX_Pusage         ((char*)"UBX_H ");
ProcessorUsage UDP_Susage         ((char*)"UDP_S ");
ProcessorUsage DACusage           ((char*)"DAC   ");
ProcessorUsage MACHusage          ((char*)"MACH  ");
ProcessorUsage LEDSusage          ((char*)"LEDS  ");
ProcessorUsage ESP32usage         ((char*)"ESP32 ");
const uint8_t numCpuUsageTasks = 17;
ProcessorUsage* cpuUsageArray[numCpuUsageTasks] = { 
  &BNOusage, &GPS1usage, &GPS2usage, &PGNusage, &ASusage, &NTRIPusage,
  &RS232usage, &LOOPusage, &IMU_Husage, &NMEA_Pusage, &RTKusage, &UBX_Pusage,
  &UDP_Susage, &DACusage, &MACHusage, &LEDSusage, &ESP32usage
};
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

//#include "reset.h"    // no on board buttons for reset
//const uint8_t RESET_BTN = A14;          // A13 on Matt's v4.0 test board, A14 ununsed on v5.0a
//RESET teensyReset(RESET_BTN, 2000, LED_BUILTIN);           // reset.h - btn IO, factory reset PERIOD (ms), led IO 

#include "Encoder.h"
Encoder encoder(KICKOUT_D_PIN, KICKOUT_A_PIN);    // read single or double input values in Autosteer.ino

#include <ADC.h>
#include <ADC_util.h>
ADC* teensyADC = new ADC();                        // 16x oversampling medium speed 12 bit A/D object (set in Autosteer.ino)
ADS1115_lite ads1115(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115
const int16_t ANALOG_TRIG_THRES = 100;
const uint8_t ANALOG_TRIG_HYST = 10;

#include "BNO_RVC.h"
BNO_RVC BNO;                                            // Roomba Vac mode for BNO085

#include "Eth_UDP.h"
Eth_UDP UDP = Eth_UDP();

#include "clsPCA9555.h" // https://github.com/nicoverduin/PCA9555
PCA9555 outputs(0x20);  // 0x20 - I2C addr (A0-A2 grounded), interrupt pin causes boot loop
#include "machine.h"
MACHINE machine;      // also used for v4 as it suppresses machine PGN debug messages
const uint8_t pcaOutputPinNumbers[8] = { 1, 0, 12, 15, 9, 8, 6, 7 };    // all 8 PCA9555 section/machine output pin numbers on v5.0a
const uint8_t pcaInputPinNumbers[]  = { 14, 13, 11, 10, 2, 3, 4, 5 };   // all 8 PCA9555 section/machine output "sensing" pin numbers on v5.0a

#include "zNMEA.h"
NMEAParser<4> nmeaParser;
bool nmeaDebug = 0, nmeaDebug2 = 0, extraCRLF;

#include "zUBXParser.h"
UBX_Parser ubxParser;

#include "zFUSEImu.h"
FUSE_Imu fuseImu;

bool USB1DTR = false;               // to track bridge mode state
bool USB2DTR = false;
uint32_t GPS1BAUD;                  // to track baud changes for bridge mode
uint32_t GPS2BAUD;

#define PANDA_SINGLE 1
#define PAOGI_DUAL 0
bool startup = false;
//elapsedMillis gpsLostTimer;
elapsedMillis LEDTimer;
elapsedMillis imuPandaSyncTimer;
bool posReady, gpsActive, imuPandaSyncTrigger;
bool ggaTimeout, relposnedTimeout;
//bool SerialGPSactive, SerialGPS2active;
uint32_t dualTime;

//constexpr int buffer_size = 512;
uint8_t GPS1rxbuffer[128];      // seems large enough
uint8_t GPS1txbuffer[256];      // large enough for 256 byte AgIO NTRIP packet
uint8_t GPS2rxbuffer[128];      // seems large enough
uint8_t GPS2txbuffer[256];      // large enough for 256 byte AgIO NTRIP packet
uint8_t RTKrxbuffer[64];        // don't know what size is needed, larger buffer if GPS baud is lower then RTK radio baud
#ifdef AIOv5
  uint8_t RS232txbuffer[256];   // large enough to hold a few NMEA sentences as ext terminal bauds are usually slow
  //uint8_t RS232rxbuffer[256]; // not needed unless custom rs232 rx code is added
  uint8_t ESP32rxbuffer[256];   // don't know what size is needed
  uint8_t ESP32txbuffer[256];   // don't know what size is needed
#endif

extern "C" uint32_t set_arm_clock(uint32_t frequency);  // required prototype to set CPU speed




