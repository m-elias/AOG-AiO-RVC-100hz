/*
Used Ace repo code and adapted for AiO v4.x/5.0a RVC 100hz
- Started with code from Teensy "Nav" Ace module https://github.com/farmerbriantee/Ace/tree/master/Hardware/Ace
- Added code bits from old AIO v4 I2C firmware https://github.com/AgHardware/Boards/tree/main/Boards/TeensyModules/AIO%20Micro%20v4/Firmware/Autosteer_gps_teensy_v4
- Much many new original code written for performance monitoring, LED control, section control etc

Single
- Save BNO reading 60ms before next GGA/GNS
	- roll/heading will be ready for PANDA later
	- if no BNO, prep vars with 0xFFFF heading, 0 roll/yaw/pitch
- Once GGA/GNS arrives (if !useDual)
	- build PANDA msg and send out
	- Otherwise if useDual, wait for relposned in main loop()

Dual
- if relposned arrives
	- set useDual for duration of runtime
- Each time new GGA/GNS & relposned arrive
	- if fix/diffsol/posvalid all good
		- calc roll from dual baseline/relposD
			- if carrsoln is not full RTK "wind down" dual roll by x0.9 each GPS update
	- Send paogi

Machine/Section outputs
- only supported by v5.0a


To-do
- consolidate all EEPROM addrs in one place?
    - Ethernet, Autosteer, machine
- test/fix autosteer watch dog timeout from lost comms
- write piezo class
- expand machine/PCA9555 to monitor output pins with input pins
- add analog PCB ID input
- use 2nd Eth jack LED for something?

- Testing !!!
  - pressure/current inputs should be scaled the same as old firmware, only bench tested by Matt
  - Single/IMU PANDA calcs should be the same as Ace branch
  - Dual PAOGI calcs should be the same as old I2C AIO firmware



See HWv??.h for hardware (board specfic) definitions (IO & Serial)
See common.h for library & other variable definitions
See PGN.ino for PGN parsing
*/

// pick only one or the other board file
#include "HWv50a.h"
//#include "HWv4x.h"

const uint8_t encoderType = 1;  // 1 - single input
                                // 2 - dual input (quadrature encoder), uses Kickout_A (Pressure) & Kickout_D (Remote) inputs

#include "common.h"

//#include "JD_DAC.h"   // experimental JD 2 track DAC steering & SCV/remote hyd control
//JD_DAC jdDac(Wire1, 0x60);

elapsedMillis bufferStatsTimer = 3000;
uint32_t testCounter;
bool printCpuUsages = false;
bool printStats = false;

void setup()
{
  #ifdef AIOv50a
    pinMode(PIEZO1, OUTPUT);
    pinMode(PIEZO2, OUTPUT);
    digitalWrite(PIEZO1, HIGH);
    digitalWrite(PIEZO2, HIGH);
  #endif

    //Serial.begin(115200);                   // Teensy doesn't need it
  Serial.print("\r\n\n\n*********************\r\nStarting setup...\r\n");
  Serial.print(inoVersion);
  LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::PWR_ON);

  setCpuFrequency(150 * 1000000);           // Set CPU speed to 600mhz, 450mhz is also a good choice(?), setup.ino
  serialSetup();                            // setup.ino
  parserSetup();                            // setup.ino
  BNO.begin(SerialIMU);                     // BNO_RVC.cpp

  //#ifdef AIOv50a
    if (outputs.begin()) {
      Serial << "\r\nSection outputs (PCA9555) detected (8 channels, low side switching)";   // clsPCA9555.cpp
      machine.init(&outputs, pcaOutputPinNumbers, 100);                                      // mach.h
    } else Serial << "\r\n*** Section outputs (PCA9555) NOT detected! ***";
  //#endif

  if (UDP.init())                           // Eth_UDP.h
    LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::ETH_READY);
  else
    LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::NO_ETH);

  autosteerSetup();                         // Autosteer.ino

  Serial.println("\r\n\nEnd of setup, waiting for GPS...\r\n"); 
  delay(1);
  resetStartingTimersBuffers();             // setup.ino
}



void loop()
{
  MACHusage.timeIn();
  machine.watchdogCheck();                 // machine.h, run machine class for v4.x to suppress unprocessed PGN messages, also reduces #ifdefs
  MACHusage.timeOut();

  checkForPGNs();                           // zPGN.ino, check for AgIO or SerialESP32 Sending PGNs
  PGNusage.timeOut();

  autoSteerUpdate();                        // Autosteer.ino, update AS loop every 10ms (100hz) regardless of whether there is a BNO installed
  udpNMEA();                                // check for NMEA via UDP
  udpNtrip();                               // check for RTCM via UDP (AgIO NTRIP client)
    
  if (SerialRTK.available()) {              // Check for RTK Radio RTCM data
    SerialGPS->write(SerialRTK.read());     // send to GPS1
    LEDs.queueBlueFlash(LED_ID::GPS);
  }

  #ifdef AIOv50a
  RS232usage.timeIn();
  if (SerialRS232->available()) {           // Check for RS232 data
    Serial.write(SerialRS232->read());      // just print to USB for testing
  }
  RS232usage.timeOut();
  #endif

  BNOusage.timeIn();
  if (BNO.read()) {                         // there should be new data every 10ms (100hz)
    //Serial.print("\r\nBNO");
    bnoStats.incHzCount();
    bnoStats.update(1);
  }
  BNOusage.timeOut();

  // wait 40 msec (F9P) from prev GGA update, then update imu data for next PANDA sentence
  if (imuPandaSyncTrigger && imuPandaSyncTimer >= 40) {
    prepImuPandaData();
    imuPandaSyncTrigger = false;       // wait for next GGA update before resetting imuDelayTimer again
    //Serial.println();
  }



  // **** "Right" Dual or Single GPS1 (position) ****
  GPS1usage.timeIn();
  int16_t gps1Available = SerialGPS->available();
  if (gps1Available)    // "if" is very crucial here, using "while" causes BNO overflow
  {
    if (gps1Available > buffer_size - 50) {   // this should not trigger except for at boot up
      SerialGPS->clear();
      Serial.print((String)"\r\n" + millis() + " *** SerialGPS buffer cleared! ***");
      return;
    }
    gps1Stats.update(gps1Available);

    uint8_t gps1Read = SerialGPS->read();
    nmeaParser << gps1Read;
    
    /*#ifdef AIOv50a
      GPS1usage.timeOut();
      RS232usage.timeIn();
      SerialRS232->write(gps1Read);
      RS232usage.timeOut();
    #endif*/

    //Serial.write(gps1Read);
    //Serial.print((String)"\nSerialGPS update " + SerialGPS->available() + " " + millis() + " d:" + (char)gps1Read);
  }
  GPS1usage.timeOut();


  // **** "Left" Dual GPS2 (heading) ****
  GPS2usage.timeIn();
  int16_t gps2Available = SerialGPS2->available();
  if (gps2Available)
  {
    if (gps2Available > buffer_size - 50) {   // this should not trigger except for at boot up
      SerialGPS2->clear();
      Serial.print((String)"\r\n" + millis() + " *** SerialGPS2 buffer cleared! ***");
      return;
    }
    gps2Stats.update(gps2Available);

    uint8_t gps2Read = SerialGPS2->read();
    ubxParser.parse(gps2Read);

    /*#ifdef AIOv50a
      GPS2usage.timeOut();
      RS232usage.timeIn();
      SerialRS232->write(gps2Read);
      RS232usage.timeOut();
    #endif*/
  }
  GPS2usage.timeOut();


  // **** For DUAL mode ****
  if (ubxParser.relPosNedReady && ggaReady) {   // if in Dual mode, and both GGA & relposNED are ready
      buildPandaOrPaogi(PAOGI);                 // build a PAOGI msg
      ubxParser.relPosNedReady = false;         // reset for next relposned trigger
      ggaReady = false;
    }

  // if either GGA or relposNED are late, don't use the old msgs, print warning only once per cycle
  if (ubxParser.useDual) {
    if ((ggaReady ^ ubxParser.relPosNedReady)) {    // true only if they are different from each other (XOR)
      if (imuPandaSyncTimer > 15 && ubxParser.relPosTimer > 15) {

        Serial.print("\r\n**************************************************"); Serial.print(millis());
        if (ggaReady) {
          Serial.print("\r\n*** relposNED was missed, late or low quality! ***");
          //Serial.print("\r\n****************** carrSoln:  "); Serial.print(ubxParser.ubxData.carrSoln); Serial.print(" ******************");
        }
        
        if (ubxParser.relPosNedReady) Serial.print("\r\n************* GGA was missed or late! ************");

        Serial.print("\r\n**************************************************\r\n");
        ggaReady = false;
        ubxParser.relPosNedReady = false;
      }
    }
  }


  // *************************************************************************************************
  // ************************************* other update routines *************************************
  // *************************************************************************************************


  // this is only for dual stats monitoring
  if (dualTime != ubxParser.ubxData.iTOW)
  {
    //itowVsHandleTime = micros();
    gps2Stats.incHzCount();
    relJitterStats.update(ubxParser.msgPeriod);
    relTtrStats.update(ubxParser.msgReadTime);
    //Serial.print(millis()); Serial.print(" New relposned update "); Serial.println(ubxParser.ubxData.iTOW - dualTime);
    dualTime = ubxParser.ubxData.iTOW;
  }
  
  if (bufferStatsTimer > 5000) printTelem();
  
  LEDSusage.timeIn();
  LEDs.updateLoop();                  // LEDS.h
  LEDSusage.timeOut();
  
  checkUSBSerial();                   // debug.ino
  speedPulse.update();                // misc.h

  #ifdef RESET_H
    teensyReset.update();             // reset.h
  #endif

  // to count loop hz & get baseline cpu "idle" time
  LOOPusage.timeIn();
  testCounter++;
  LOOPusage.timeOut();
} // end of loop()




