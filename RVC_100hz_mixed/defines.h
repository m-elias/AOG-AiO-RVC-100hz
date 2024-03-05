#include <stdint.h>
#include <Streaming.h>
#include "Eth_UDP.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <ADC.h>
#include <ADC_util.h>
#include "zADS1115.h"
#include "BNO_RVC.h"
#include "zNMEA.h"
#include "reset.h"
#include "misc.h"
#include "Encoder.h"
#include "zUBXParser.h"

#ifdef AIOv50a
  //#include <Adafruit_NeoPixel.h>

  #include "clsPCA9555.h" // https://github.com/nicoverduin/PCA9555
  PCA9555 outputs(0x20);  // 0x20 - I2C addr (A0-A2 grounded), interrupt pin causes boot loop

  #include "machine.h"
  MACHINE machine;

  #include "JD_DAC.h"   // Matt's experimental DAC steering
  JD_DAC jdDac(Wire1, 0x60);
#endif

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
const uint8_t numCpuUsageTasks = 15;
ProcessorUsage* cpuUsageArray[numCpuUsageTasks] = { 
  &BNOusage, &GPS1usage, &GPS2usage, &PGNusage, &ASusage, &NTRIPusage,
  &RS232usage, &LOOPusage, &IMU_Husage, &NMEA_Pusage, &RTKusage, &UBX_Pusage,
  &UDP_Susage, &DACusage, &MACHusage
};

HighLowHzStats gps2Stats;
HighLowHzStats gps1Stats;
HighLowHzStats relJitterStats;
HighLowHzStats relTtrStats;

//Reset teensyReset(RESET_BTN, 2000, LED_BUILTIN);           // reset.h - btn IO, factory reset PERIOD (ms), led IO 

SpeedPulse speedPulse(SPEEDPULSE_PIN, SPEEDPULSE10_PIN);     // misc.h
Encoder encoder(KICKOUT_D_PIN, KICKOUT_A_PIN);

ADC* teensyADC = new ADC();                        //16x oversampling medium speed 12 bit A/D object (set in Autosteer.ino)
ADS1115_lite ads1115(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115
const int16_t ANALOG_TRIG_THRES = 100;
const uint8_t ANALOG_TRIG_HYST = 10;

elapsedMillis gpsLostTimer;
elapsedMillis LEDTimer;
elapsedMillis imuPandaSyncTimer;

bool ggaReady = false;
bool SerialGPSactive, SerialGPS2active;

uint32_t dualTime;

BNO_RVC BNO;             //Roomba Vac mode for BNO085

Eth_UDP UDP = Eth_UDP();

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype to set CPU speed

NMEAParser<2> nmeaParser;         // A parser is declared with 3 handlers at most
UBX_Parser ubxParser;
bool startup = true;

constexpr int buffer_size = 256;
uint8_t GPSrxbuffer[buffer_size];    // Extra GPS1 serial rx buffer
uint8_t GPStxbuffer[buffer_size];    // Extra GPS1 serial tx buffer
uint8_t GPS2rxbuffer[buffer_size];   // Extra GPS2 serial rx buffer
uint8_t GPS2txbuffer[buffer_size];   // Extra GPS2 serial tx buffer
uint8_t RTKrxbuffer[buffer_size];    // Extra RTK serial rx buffer
uint8_t RS232txbuffer[buffer_size];  // Extra RS232 serial tx buffer

