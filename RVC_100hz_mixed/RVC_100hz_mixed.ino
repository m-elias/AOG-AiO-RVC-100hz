/*

See HWv??.h for hardware (board specfic) definitions (IO & Serial)
See common.h for library & other variable definitions common to all boards
See debug.ino for optional debug commands
See PGN.ino for PGN parsing
See notes.ino for additional information

*/


// pick only one board file
//#include "HWv4x.h"
//#include "HWv50a.h"
#include "HWv50d.h"

const uint8_t encoderType = 1;  // 1 - single input
                                // 2 - dual input (quadrature encoder), uses Kickout_A (Pressure) & Kickout_D (Remote) inputs
                                // 3 - variable duty cycle, for future updates

#include "common.h"

//#include "JD_DAC.h"   // experimental JD 2 track DAC steering & SCV/remote hyd control
//JD_DAC jdDac(I2C_WIRE, 0x60, &Serial);
//#include "OGX.h"
//OpenGradeX grade;

void setup()
{
  //Serial.begin(115200);                   // Teensy doesn't need it
  Serial.print("\r\n\n\n*********************\r\nStarting setup...\r\n");
  Serial.print(inoVersion);
  LEDs.init();
  LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::PWR_ON);

  setCpuFrequency(600 * 1000000);           // Set CPU speed, default is 600mhz, 150mhz still seems fast enough, setup.ino
  serialSetup();                            // setup.ino
  parserSetup();                            // setup.ino
  BNO.begin(SerialIMU);                     // BNO_RVC.cpp

  // v5 has machine outputs, v4 fails outputs.begin so machine is also not init'd
  if (outputs.begin(I2C_WIRE)) {            // clsPCA9555.cpp
    Serial.print("\r\nSection outputs (PCA9555) detected (8 channels, low side switching)");
    machine.init(&outputs, pcaOutputPinNumbers, pcaInputPinNumbers, 100); // mach.h
  }

  if (UDP.init())                           // Eth_UDP.h
    LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::ETH_READY);
  else
    LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::NO_ETH);

  autosteerSetup();                         // Autosteer.ino

  #ifdef OGX_H
    OGX_Setup();
  #endif

  Serial.println("\r\n\nEnd of setup, waiting for GPS...\r\n"); 
  delay(1);
  resetStartingTimersBuffers();             // setup.ino
}

void loop()
{
  checkForPGNs();                           // zPGN.ino, check for AgIO or SerialESP32 Sending PGNs
  PGNusage.timeOut();

  #ifdef OGX_H
    checkForOGXPackets();
    grade.updateLoop();
  #endif

  autoSteerUpdate();                        // Autosteer.ino, update AS loop every 10ms (100hz) regardless of whether there is a BNO installed
  udpNMEA();                                // check for NMEA via UDP
  udpNtrip();                               // check for RTCM via UDP (AgIO NTRIP client)

  if (SerialRTK.available()) {              // Check for RTK Radio RTCM data
    uint8_t rtcmByte = SerialRTK.read();
    if (!USB1DTR) SerialGPS1.write(rtcmByte);    // send to GPS1
    #ifdef OGX_H
      if (!USB2DTR) SerialGPS2.write(rtcmByte);    // send to GPS2
    #endif
    LEDs.queueBlueFlash(LED_ID::GPS);
  }

  #ifdef AIOv5
    RS232usage.timeIn();
    if (SerialRS232.available()) {           // Check for RS232 data
      Serial.write(SerialRS232.read());      // just print to USB for testing
    }
    RS232usage.timeOut();
  #endif

  BNOusage.timeIn();
  if (BNO.read()) {                         // there should be new data every 10ms (100hz)
    bnoStats.incHzCount();
    bnoStats.update(1);                     // 1 dummy value
    //LEDs.setScheduler(PORT_ID::IMU_ID, 0, 9);
  }
  BNOusage.timeOut();

  // wait 40 msec (F9P) from prev GGA update, then update imu data for next PANDA sentence
  if (imuPandaSyncTrigger && imuPandaSyncTimer >= 40) {
    prepImuPandaData();
    imuPandaSyncTrigger = false;       // wait for next GGA update before resetting imuDelayTimer again
  }



  // ******************* "Right" Single GPS1 (position) *******************
  GPS1usage.timeIn();
  if (!USB1DTR)                 // carry on like normal
  {
    uint16_t gps1Available = SerialGPS1.available();
    if (gps1Available)    // "if" is very crucial here, using "while" causes BNO overflow
    {
      if (gps1Available > sizeof(GPS1rxbuffer) - 10) {   // this should not trigger except maybe at boot up
        SerialGPS1.clear();
        Serial.print((String)"\r\n" + millis() + " *** SerialGPS1 buffer cleared! ***");
        return;
      }
      gps1Stats.update(gps1Available);

      uint8_t gps1Read = SerialGPS1.read();
      if (nmeaDebug) Serial.write(gps1Read);
      
      #ifndef OGX_H
        nmeaParser << gps1Read; // for normal, autosteer setup
      #else
        //grade.nmeaInput(gps1Read);  // GPS1 needs to output GGA/GNS & VTG which is sent to OGX for blade position
        nmeaParser << gps1Read; // for normal, autosteer setup
      #endif

      //LEDs.setScheduler(PORT_ID::GPS1_ID, 5, 35);
      
      #ifdef AIOv5
        GPS1usage.timeOut();
        RS232usage.timeIn();
        SerialRS232.write(gps1Read);
        RS232usage.timeOut();
      #endif
    }
    GPS1usage.timeOut();
  }

  #if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    else {                                // in SerialUSB1<->SerialGPS1 bridge mode, for connecting via u-center
      if (SerialGPS1.available()) {
        while (SerialGPS1.available()) {     // seems necessary to keep sentences/packets grouped as tight as possible
          SerialUSB1.write(SerialGPS1.read());
          //Serial.write(SerialGPS1.read());
        }
      }
      if (SerialUSB1.available()) {           // seems necessary to ensure UBX msgs from U-Center aren't interrupted by RTCM data (xbee or ntrip)
        while (SerialUSB1.available()) {
          SerialGPS1.write(SerialUSB1.read());
        }
      }
    }
  #endif



  // ******************* "Left" GPS2 (OGX Blade) *******************
  GPS2usage.timeIn();
  if (!USB2DTR)                 // carry on like normal
  {
    uint16_t gps2Available = SerialGPS2.available();
    if (gps2Available)
    {
      if (gps2Available > sizeof(GPS2rxbuffer) - 10) {   // this should not trigger except maybe at boot up
        SerialGPS2.clear();
        Serial.print((String)"\r\n" + millis() + " *** SerialGPS2 buffer cleared! ***");
        return;
      }
      gps2Stats.update(gps2Available);

      uint8_t gps2Read = SerialGPS2.read();
      if (nmeaDebug2) Serial << "(" << byte(gps2Read) << ")";
      #ifndef OGX_H
        ubxParser.parse(gps2Read);  // for dual F9P autosteer
      #else
        grade.nmeaInput(gps2Read);  // no UBX in this version, GPS2 outputs GGA/GNS & VTG which is sent to OGX for blade position
        //nmeaParser << gps2Read; // autosteer with GPS2
      #endif
    }
  }

  #if defined(USB_TRIPLE_SERIAL)
    else {                                // in SerialUSB2<->SerialGPS2 bridge mode, for connecting via u-center
      if (SerialGPS2.available()) {
        while (SerialGPS2.available()) {     // seems necessary to keep sentences/packets grouped as tight as possible
          SerialUSB2.write(SerialGPS2.read());
        }
      }
      if (SerialUSB2.available()) {           // seems necessary to ensure UBX msgs from U-Center aren't interrupted by RTCM data (xbee or ntrip)
        while (SerialUSB2.available()) {
          SerialGPS2.write(SerialUSB2.read());
        }
      }
    }
  #endif

  /*#ifdef AIOv5
    GPS2usage.timeOut();
    RS232usage.timeIn();
    SerialRS232.write(gps2Read);
    RS232usage.timeOut();
  #endif*/
  GPS2usage.timeOut();


  // ******************* For SINGLE/RIGHT *******************
  if (imuPandaSyncTimer > 50 && startup) {   // to make sure old data isn't sent to AOG
    if (posReady) {
      posReady = 0;
      Serial << "\r\n" << millis() << " **Position data expired**\r\n";
    }
  
    if (extraCRLF && nmeaDebug) {
      Serial.print("\r\n");
      extraCRLF = false;
    }
  }

  if (imuPandaSyncTimer > 150 && startup && gpsActive) {
    imuPandaSyncTimer -= 100;
    ggaMissed++;
    if (nmeaDebug) Serial.println();
    Serial << "\r\n" << millis() << " *** position was missed or late! *** (" << ggaMissed << ")\r\n";
    posReady = false;
    ubxParser.relPosNedReady = false;
  }

  // ******************* For DUAL LEFT *******************
  if (ubxParser.relPosNedReady && posReady) {   // if both GGA & relposNED are ready
    buildPandaOrPaogi(PAOGI_DUAL);              // build a PAOGI msg
    ubxParser.relPosNedReady = false;           // reset for next relposned trigger
    ubxParser.relPosNedRcvd = false;
    posReady = false;
  }

  if (ubxParser.relPosTimer > 50 && ubxParser.relPosNedReady && startup) {    // to make sure old data isn't sent to AOG
    ubxParser.relPosNedReady = 0;
    if (!ubxParser.firstHeadingDetected) {
      Serial.print("\r\n**Heading data expired**\r\n");
      ubxParser.firstHeadingDetected = 0;
    }
  }

  if (ubxParser.relPosTimer > 150 && ubxParser.useDual && startup) {
    ubxParser.relPosTimer -= 100;
    ubxParser.relMissed++;
    if (nmeaDebug) Serial.println();
    Serial << "\r\n" << millis() << " *** heading was missed or late! *** (" << ubxParser.relMissed << ")\r\n";
    ubxParser.clearCount();
    posReady = false;
    ubxParser.relPosNedReady = false;
  }

  /*if (ubxParser.pvtTimer > 150) {
    ubxParser.pvtTimer -= 100;
    Serial.print("\r\n\n"); Serial.print(millis()); Serial.print(" ");
    Serial.print("                 *** PVT was missed or late! ***\r\n");
  }*/

  // *************************************************************************************************
  // ************************************* UPDATE OTHER STUFF *************************************
  // *************************************************************************************************


  // this is only for dual stats monitoring
  if (dualTime != ubxParser.ubxData.iTOW)
  {
    gps2Stats.incHzCount();
    relJitterStats.update(ubxParser.msgPeriod);
    relTtrStats.update(ubxParser.msgReadTime);
    dualTime = ubxParser.ubxData.iTOW;
  }
  
  if (bufferStatsTimer > 5000) printTelem();
  
  LEDSusage.timeIn();
  LEDs.updateLoop();                  // LEDS.h
  LEDSusage.timeOut();

  MACHusage.timeIn();
  machine.watchdogCheck();            // machine.h, run machine class for v4.x to suppress unprocessed PGN messages, also reduces #ifdefs
  MACHusage.timeOut();

  checkUSBSerial();                   // debug.ino
  speedPulse.update();                // misc.h

  #if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    static bool prevUSB1DTR;
    USB1DTR = SerialUSB1.dtr(); 
    if (USB1DTR != prevUSB1DTR) {
      Serial << "\r\n**SerialUSB1 " << (USB1DTR ? "bridged with GPS1" : "disconnected");
      if (USB1DTR) {
        if (SerialUSB1.baud() == GPS1BAUD) Serial << ", baud set at " << baudGPS << " (default)";
      } else {
        if (GPS1BAUD != baudGPS){
          SerialGPS1.begin(baudGPS);
          GPS1BAUD = baudGPS;
          Serial << ", baud reverted back to default " << GPS1BAUD;
        }
      }
      prevUSB1DTR = USB1DTR;
    }

    if (USB1DTR) {
      if (SerialUSB1.baud() != GPS1BAUD) {
        SerialGPS1.begin(SerialUSB1.baud());
        GPS1BAUD = SerialUSB1.baud();
        Serial << "\r\n**GPS1 baud changed to " << GPS1BAUD;
        if (GPS1BAUD == baudGPS) Serial << " (default)";
      }
    }
  #endif

  #if defined(USB_TRIPLE_SERIAL)
    static bool prevUSB2DTR;
    USB2DTR = SerialUSB2.dtr(); 
    if (USB2DTR != prevUSB2DTR) {
      Serial << "\r\n**SerialUSB2 " << (USB2DTR ? "bridged with GPS2" : "disconnected");
      if (USB2DTR) {
        if (SerialUSB2.baud() == GPS2BAUD) Serial << ", baud set at " << baudGPS << " (default)";
      } else {
        if (GPS2BAUD != baudGPS){
          SerialGPS2.begin(baudGPS);
          GPS2BAUD = baudGPS;
          Serial << ", baud reverted back to default " << GPS2BAUD;
        }
      }
      prevUSB2DTR = USB2DTR;
    }

    if (USB2DTR) {
      if (SerialUSB2.baud() != GPS2BAUD) {
        SerialGPS2.begin(SerialUSB2.baud());
        GPS2BAUD = SerialUSB2.baud();
        Serial << "\r\n**GPS2 baud changed to " << GPS2BAUD;
        if (GPS2BAUD == baudGPS) Serial << " (default)";
      }
    }
  #endif

  #ifdef RESET_H
    teensyReset.update();             // reset.h
  #endif

  // to count loop hz & get baseline cpu "idle" time
  LOOPusage.timeIn();
  testCounter++;
  LOOPusage.timeOut();

} // end of loop()

#ifdef OGX_H
  #ifdef JD_DAC_H
    void updateDacChannel4Output() {
      jdDac.ch4Enable(true);
      jdDac.ch4GradeOutput(grade.getAnalog1());
      //Serial << "\r\nGrade output: " << grade.getAnalog1();
    }
  #else
    void updatePwmOutput() {
      // add code here to output PWM to Cytron
    }
  #endif

  void forwardNtripData(char* _ntripBuffer, uint16_t _len) {
    if (!USB1DTR) SerialGPS1.write(_ntripBuffer, _len);    // send to GPS1
    if (!USB2DTR) SerialGPS2.write(_ntripBuffer, _len);    // send to GPS2
  }
#endif


