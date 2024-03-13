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


To-do
- consolidate all EEPROM addrs in one place?
    - Ethernet, Autosteer, machine
- test autosteer watch dog timeout from lost comms
- verify Module Hello repy telemetry is working (display in AgIO advanced view)

- Testing !!!
  - pressure/current inputs should be scaled the same as old firmware, only bench tested by Matt
  - Single/IMU PANDA calcs should be the same as Ace branch
  - Dual PAOGI calcs should be the same as old I2C AIO firmware



See hw.h for hardware (board specfic) definitions (IO & Serial)
See defines.h for library & other variable definitions
See PGN.ino for PGN parsing

*/

// pick only one or the other
#include "HWv50a.h"
//#include "HWv4x.h"

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
  LEDS.setPwrEthLED(AIO_LEDS::PWR_ON);

  setCpuFrequency(600 * 1000000);           // Set CPU speed to 600mhz, 450mhz is also a good choice(?), setup.ino
  serialSetup();                            // setup.ino
  parserSetup();                            // setup.ino
  BNO.begin(SerialIMU);                     // BNO_RVC.cpp

  #ifdef AIOv50a
    if (outputs.begin()) {
      Serial << "\r\nSection outputs (PCA9555) detected (8 channels, low side switching)";   // clsPCA9555.cpp
      machine.init(&outputs, pcaOutputPinNumbers, 100);                                      // mach.h
    } else Serial << "\r\n*** Section outputs (PCA9555) NOT detected! ***";
  #endif

  if (UDP.init())                           // Eth_UDP.h
    LEDS.setPwrEthLED(AIO_LEDS::ETH_READY);
  else
    LEDS.setPwrEthLED(AIO_LEDS::NO_ETH);

  autosteerSetup();                         // Autosteer.ino

  //#ifdef JD_DAC_H
    //jdDac.update();
  //#endif

  Serial.println("\r\n\nEnd of setup, waiting for GPS...\r\n"); 
  delay(1);
  resetStartingTimersBuffers();
}



void loop()
{
  #ifdef JD_DAC_H
    //jdDac.update(); // should be in AS update, but left here for in case it's needed during testing
  #endif

  #ifdef AIOv50a
    MACHusage.timeIn();
    machine.watchdogCheck();                // machine.h
    MACHusage.timeOut();
  #endif

  PGNusage.timeIn();
  checkForPGNs();                           // check for AgIO Sending PGNs, AgIO sends autosteer data at ~10hz
  PGNusage.timeOut();
  
  autoSteerUpdate();                        // Autosteer.ino, update AS loop every 10ms (100hz) regardless of whether there is a BNO installed
  udpNMEA();                                // check for NMEA via UDP
  udpNtrip();                               // check for RTCM via UDP (AgIO NTRIP client)
    
  if (SerialRTK.available()) {               // Check for RTK Radio RTCM data
    SerialGPS->write(SerialRTK.read());
    LEDS.rtcmReceived();
  }

  RS232usage.timeIn();
  if (SerialRS232->available()) {               // Check for RTK Radio RTCM data
    Serial.write(SerialRS232->read());
  }
  RS232usage.timeOut();

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
  



  // *******************************
  // **** other update routines ****
  // *******************************
  if (bufferStatsTimer > 5000) printTelem();
  
  LEDSusage.timeIn();
  LEDS.updateLoop();
  LEDSusage.timeOut();
  
  checkUSBSerial();
  speedPulse.update();

  #ifdef RESET_H
    teensyReset.update();
  #endif

  // to count loop hz & get baseline cpu "idle" time
  LOOPusage.timeIn();
  testCounter++;
  LOOPusage.timeOut();
} // end of loop()


void resetStartingTimersBuffers()
{
  //machine.watchdogTimer = 0;
  SerialGPS->clear();
  SerialGPS2->clear();
  if (BNO.isActive) while (!BNO.read(true));
  machine.watchdogTimer = 0;
  startup = true;
}


void checkUSBSerial()
{
  if (Serial.available())
  {
    uint8_t usbRead = Serial.read();
    if (usbRead == 'r')
    {
      Serial.print("\r\n\n* Resetting hi/lo stats *");
      gps1Stats.resetAll();
      gps2Stats.resetAll();
      relJitterStats.resetAll();
      relTtrStats.resetAll();
      bnoStats.resetAll();
    }
    else if (usbRead == 'n')
    {
      nmeaDebug = !nmeaDebug;
    }
    else if (usbRead == 'c')
    {
      printCpuUsages = !printCpuUsages;
    }
    else if (usbRead == 's')
    {
      printStats = !printStats;
    }
    else if (usbRead == 'm')
    {
      machine.debugLevel = max(Serial.read() - '0', 5);
      Serial.print((String)"\r\nMachine debugLevel: " + machine.debugLevel);
    }
    else if (usbRead >= '0' && usbRead <= '5')
    {
      LEDS.setGpsLED(usbRead - '0', true);
    }
  }
}

void printTelem()
{
  if (printStats)
  {
    gps1Stats.printStatsReport((char*)"GPS1");
    gps2Stats.printStatsReport((char*)"GPS2");
    relJitterStats.printStatsReport((char*)"RELj");
    relTtrStats.printStatsReport((char*)"RELr");
    bnoStats.printStatsReport((char*)"BNO");
  }

  if (printCpuUsages)
  {
    uint32_t rs232report = RS232usage.reportAve();
    uint32_t baselineProcUsage = LOOPusage.reportAve();
    uint32_t dacReport = DACusage.reportAve();
    Serial.print("\r\n\nLoop   cpu: "); printCpuPercent(baselineProcUsage);
    Serial.print(" "); Serial.print(testCounter / bufferStatsTimer); Serial.print("kHz"); // up to 400k hits/s
    Serial.print("\r\nBNO_R  cpu: "); printCpuPercent(cpuUsageArray[0]->reportAve(baselineProcUsage));
    Serial.print("\r\nGPS1   cpu: "); printCpuPercent(GPS1usage.reportAve(baselineProcUsage));// - rs232report);
    Serial.print("\r\nGPS2   cpu: "); printCpuPercent(GPS2usage.reportAve(baselineProcUsage));
    //Serial.print("\r\nRadio  cpu: "); printCpuPercent(RTKusage.reportAve(baselineProcUsage));
    Serial.print("\r\nPGN    cpu: "); printCpuPercent(PGNusage.reportAve(baselineProcUsage));
    Serial.print("\r\nAS     cpu: "); printCpuPercent(ASusage.reportAve() - dacReport);
    Serial.print("\r\nNTRIP  cpu: "); printCpuPercent(NTRIPusage.reportAve());  // uses a timed update, virtually no extra time penalty
    Serial.print("\r\nIMU_H  cpu: "); printCpuPercent(IMU_Husage.reportAve());
    Serial.print("\r\nNMEA_P cpu: "); printCpuPercent(NMEA_Pusage.reportAve());
    Serial.print("\r\nUBX_P  cpu: "); printCpuPercent(UBX_Pusage.reportAve());
    Serial.print("\r\nUDP_S  cpu: "); printCpuPercent(UDP_Susage.reportAve());
    Serial.print("\r\nLEDS   cpu: "); printCpuPercent(LEDSusage.reportAve(baselineProcUsage));
    
    #ifdef AIOv50a
      Serial.print("\r\nRS232  cpu: "); printCpuPercent(rs232report); //RS232usage is inside GPS2 "if" statement so it inccurs virtually no extra time penalty
      Serial.print("\r\nMach   cpu: "); printCpuPercent(MACHusage.reportAve(baselineProcUsage));
    #endif

    #ifdef JD_DAC_H
      Serial.print("\r\nDAC    cpu: "); printCpuPercent(dacReport);
    #endif

    Serial.println();
  }

  testCounter = 0;
  bufferStatsTimer = 0;
}

void printCpuPercent(uint32_t _time) {
  Serial.printf("%4.1f", (float)_time / 10000.0); Serial.print("%");
}


