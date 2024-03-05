/*
Combined Ace repo code and adapted for AiO v4.x RVC 100hz
- Started with code from Teensy "Nav" Ace module https://github.com/farmerbriantee/Ace/tree/master/Hardware/Ace
- Added code bits from old AIO v4 I2C firmware https://github.com/AgHardware/Boards/tree/main/Boards/TeensyModules/AIO%20Micro%20v4/Firmware/Autosteer_gps_teensy_v4



To-do
- consolidate all EEPROM addrs in one place?
    - Ethernet, Autosteer
- add dual GPS code
- test autosteer watch dog timeout from lost comms

- Testing !!!
  - pressure/current inputs should be scaled the same as old firmware, only bench tested by Matt
  - 



See hw.h for hardware definitions (IO & Serial)
See defines.h for library & other variable definitions
See PGN.ino for PGN parsing

*/

// pick only one or the other
#include "HWv50a.h"
//#include "HWv4x.h"

#include "defines.h"

#ifdef AIOv50a
const char inoVersion[] = "RVC 100hz AiO v5.0a pre-alpha - " __DATE__;
#else // AIOv4x
const char inoVersion[] = "RVC 100hz AiO v4.x pre-alpha - " __DATE__;
#endif

elapsedMillis bufferStatsTimer = 3000;
uint32_t testCounter;

void setup()
{
  ioSetup();                                // setup.ino
  teensyLedON();

  //Serial.begin(115200);                   // Teensy doesn't need it
  Serial.print("\r\n\r\n\r\n*********************\r\nStarting setup...\r\n");
  Serial.print(inoVersion);

  setCpuFrequency(600 * 1000000);           // Set CPU speed to 600mhz, 450mhz is also a good choice(?), setup.ino
  serialSetup();                            // setup.ino
  parserSetup();                            // setup.ino
  BNO.begin(SerialIMU);  // BNO_RVC.cpp

  #ifdef AIOv50a
    if (outputs.begin()) Serial << "\r\nSection outputs (PCA9555) detected (8 channels, low side switching)";   // clsPCA9555.cpp
    else Serial << "\r\n*** Section outputs (PCA9555) NOT detected! ***";
    machine.init(&outputs, pcaOutputPinNumbers, 100);                                                           // mach.h
  #endif

  autosteerSetup();                         // Autosteer.ino
  UDP.init();                               // Eth_UDP.h

  #ifdef JD_DAC_H
    jdDac.update();
  #endif

  Serial.println("\r\n\nEnd of setup, waiting for GPS...\r\n"); 
  teensyLedOFF();
  delay(1);

  resetClearStartingTimersBuffers();
}



void loop()
{
  #ifdef JD_DAC_H
    //jdDac.update(); // should be in AS update, but left here for in case it's needed during testing
  #endif

  checkForPGNs();                           // check for AgIO Sending PGNs, AgIO sends autosteer data at ~10hz
  autoSteerUpdate();                        // Autosteer.ino, update AS loop every 10ms (100hz) regardless of whether there is a BNO installed
  
  #ifdef AIOv50a
    MACHusage.timeIn();
    machine.watchdogCheck();                // machine.h
    MACHusage.timeOut();
  #endif


  // **** Check/read GPS/NMEA/IMU related inputs ****
  udpNMEA();                                // check for NMEA via UDP
  udpNtrip();                               // check for RTCM via UDP (AgIO NTRIP client)
    
  RTKusage.timeIn();
  if (SerialRTK.available())                // Check for RTK Radio RTCM data
    SerialGPS->write(SerialRTK.read());
  RTKusage.timeOut();

  BNOusage.timeIn();
  if (BNO.read()) {                         // there should be new data every 10ms (100hz)
    //Serial.print("\r\nBNO");
  }
  BNOusage.timeOut();

  // wait 40 msec (F9P) from prev GGA update, then update imu data for next PANDA sentence
  if (ggaReady && imuPandaSyncTimer >= 40) {
    prepImuPandaData();
    ggaReady = false;                     // wait for next GGA update before resetting imuDelayTimer again
    //ubxParser.relPosNedReady = false;     // it it hasn't already been reset, do it now so that it doesn't use an old relposned
    Serial.println();
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
    NMEA_Pusage.timeIn();
    nmeaParser << gps1Read;
    NMEA_Pusage.timeOut();
    
    /*#ifdef AIOv50a
      RS232usage.timeIn();
      SerialRS232->write(gpsRead);
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
    UBX_Pusage.timeIn();
    ubxParser.parse(gps2Read);
    UBX_Pusage.timeOut();

    #ifdef AIOv50a
      RS232usage.timeIn();
      SerialRS232->write(gps2Read);
      RS232usage.timeOut();
    #endif
  }
  GPS2usage.timeOut();


  if (dualTime != ubxParser.ubxData.iTOW)
  {
    //itowVsHandleTime = micros();
    gps2Stats.incHzCount();
    relJitterStats.update(ubxParser.msgPeriod);
    relTtrStats.update(ubxParser.msgReadTime);
    //Serial.print(millis()); Serial.print(" New relposned update "); Serial.println(ubxParser.ubxData.iTOW - dualTime);
    dualTime = ubxParser.ubxData.iTOW;
  }
  
  if (ubxParser.relPosNedReady && ggaReady) {    // if in Dual mode, and both GGA & relposNED are ready
      buildPandaOrPaogi(false);
      ubxParser.relPosNedReady = false;
    }



  // *******************************
  // **** other update routines ****
  // *******************************
  if (bufferStatsTimer > 5000) {
    printTelem();
  }

  if (LEDTimer > 1000){
    LEDTimer = 0;
    LEDRoutine();
  }

  speedPulse.update();
  //teensyReset.update();

  LOOPusage.timeIn();
  testCounter++;
  LOOPusage.timeOut();
  
  checkUSBSerial();

} // end of loop()


void resetClearStartingTimersBuffers()
{
  machine.watchdogTimer = 0;
  if (BNO.isActive) while (!BNO.read(true));
  SerialGPS->clear();
  SerialGPS2->clear();
  startup = true;
}


void checkUSBSerial()
{
  if (Serial.available())
  {
    if (Serial.read() == 'c')
    {
      Serial.print("\r\n\n* Resetting hi/lo stats *");
      gps1Stats.resetAll();
      gps2Stats.resetAll();
      relJitterStats.resetAll();
      relTtrStats.resetAll();
    }
  }
}

void printTelem()
{
  gps1Stats.printStatsReport((char*)"GPS1");
  gps2Stats.printStatsReport((char*)"GPS2");
  relJitterStats.printStatsReport((char*)"RELj");
  relTtrStats.printStatsReport((char*)"RELr");

  /*uint32_t rs232report = RS232usage.reportAve();
  uint32_t baselineProcUsage = LOOPusage.reportAve();
  uint32_t dacReport = DACusage.reportAve();
  Serial.print("\r\n\nLoop   cpu: "); printCpuPercent(baselineProcUsage);
  Serial.print(" "); Serial.print(testCounter / bufferStatsTimer); Serial.print("kHz"); // up to 400k hits/s
  Serial.print("\r\nBNO_R  cpu: "); printCpuPercent(cpuUsageArray[0]->reportAve(baselineProcUsage));
  Serial.print("\r\nGPS1   cpu: "); printCpuPercent(GPS1usage.reportAve(baselineProcUsage));
  Serial.print("\r\nGPS2   cpu: "); printCpuPercent(GPS2usage.reportAve(baselineProcUsage) - rs232report);
  Serial.print("\r\nRadio  cpu: "); printCpuPercent(RTKusage.reportAve(baselineProcUsage));
  Serial.print("\r\nPGN    cpu: "); printCpuPercent(PGNusage.reportAve(baselineProcUsage));
  Serial.print("\r\nAS     cpu: "); printCpuPercent(ASusage.reportAve() - dacReport);
  Serial.print("\r\nNTRIP  cpu: "); printCpuPercent(NTRIPusage.reportAve());  // uses a timed update, virtually no extra time penalty
  Serial.print("\r\nIMU_H  cpu: "); printCpuPercent(IMU_Husage.reportAve());
  Serial.print("\r\nNMEA_P cpu: "); printCpuPercent(NMEA_Pusage.reportAve());
  Serial.print("\r\nUBX_P  cpu: "); printCpuPercent(UBX_Pusage.reportAve());
  Serial.print("\r\nUDP_S  cpu: "); printCpuPercent(UDP_Susage.reportAve());
  
  #ifdef AIOv50a
    Serial.print("\r\nRS232  cpu: "); printCpuPercent(rs232report); //RS232usage is inside GPS2 "if" statement so it inccurs virtually no extra time penalty
    Serial.print("\r\nMach   cpu: "); printCpuPercent(MACHusage.reportAve(baselineProcUsage));
  #endif

  #ifdef JD_DAC_H
    Serial.print("\r\nDAC    cpu: "); printCpuPercent(dacReport);
  #endif

  Serial.println();
  */

  testCounter = 0;
  bufferStatsTimer = 0;
}

void printCpuPercent(uint32_t _time) {
  Serial.printf("%4.1f", (float)_time / 10000.0); Serial.print("%");
}



