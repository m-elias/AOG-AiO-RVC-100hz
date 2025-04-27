void checkUSBSerial()
{
  if (Serial.available())
  {
    uint8_t usbRead = Serial.read();

    if (usbRead == 'r')
    {
      Serial.print("\r\n\n* Resetting stats *");
      gps1Stats.resetAll();
      gps2Stats.resetAll();
      relJitterStats.resetAll();
      relTtrStats.resetAll();
      bnoStats.resetAll();
      ggaMissed = 0;
      ubxParser.relMissed = 0;
    }
    else if (usbRead == 'n')         // output realtime GPS position update data
    {
      uint8_t usbRead2 = 0;
      if (Serial.available()) {
        usbRead2 = Serial.peek();   // only peek in case it's not a 2, leave in Serial buffer for further processing other cmds
      }

      if (usbRead2 == '2') {
        Serial.read();              // clear the 2 out of the Serial buffer
        nmeaDebug2 = !nmeaDebug2;
        Serial.print("\r\nSetting NMEA2 debug: "); Serial.print(nmeaDebug2);
      } else {
        nmeaDebug = !nmeaDebug;
        ubxParser.debug = nmeaDebug;
        Serial.print("\r\nSetting NMEA debug: "); Serial.print(nmeaDebug);
      }
    }
    else if (usbRead == 'c')        // output cpu usage stats
    {
      printCpuUsages = !printCpuUsages;
      Serial.print("\r\nSetting CPU usage debug: "); Serial.print(printCpuUsages);
    }
    else if (usbRead == 's')        // output GPS, BNO update freq & buffer stats
    {
      printStats = !printStats;
      Serial.print("\r\nSetting Print Stats: "); Serial.print(printStats);
    }
    #ifdef AIOv5
      else if (usbRead == 'm' && Serial.available() > 0)    // set machine debug level
      {
        usbRead = Serial.read();
        if (usbRead >= '0' && usbRead <= '5') {
          machine.debugLevel = usbRead - '0';   // convert ASCII numerical char to byte
        }
        Serial.print((String)"\r\nMachine debugLevel: " + machine.debugLevel);
      }
    #endif
    else if (usbRead == 'g' && Serial.available() > 0)    // temporarily set GPS fix state according to standard GGA fix numbers (see LEDS.h, setGpsLED())
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5') {
        LEDs.setGpsLED(usbRead - '0', true);
      }
    }
    else if (usbRead == 'l' && Serial.available() > 0)    // set RGB brightness
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5') {
        LEDs.setBrightness((usbRead - '0') * 50);
        Serial.print("\r\nSetting RGB brightness: "); Serial.print((usbRead - '0') * 50);
      }
    }

    else if (usbRead == '1')      // drv9243 testing, cycle LOCK through sleep, standby, active
    {
      uint32_t t1 = micros();
      static uint8_t state = 0; // sleep
      if (state == 0){
        Serial << "\r\nLOCK is in Sleep mode, sending wake signal";
        outputs.setPin(15, 0, 1); // sets PCA9685 pin HIGH 5V, init Wake, after 1ms should be in Standby
        state = 1;
      } else if (state == 1){
        Serial << "\r\nLOCK is in Standby mode, waiting for reset, sending reset pulse";
        outputs.setPin(15, 237, 1);  // Sleep reset pulse
        state = 2;
      } else if (state ==2) {
        Serial << "\r\nLOCK is in Active mode, issuing Sleep signal";
        outputs.setPin(15, 0, 0); // sets PCA9685 pin LOW 0V, Deep Sleep
        state = 0;
      }
      uint32_t t2 = micros();
      Serial << "\r\nLOCK " << t2 - t1 << "uS";
    }

    else if (usbRead == '2')      // drv9243 testing, cycle AUX through sleep, standby, active
    {
      uint32_t t1 = micros();
      static uint8_t state = 0; // sleep
      if (state == 0){
        Serial << "\r\nAUX is in Sleep mode, sending wake signal";
        outputs.setPin(14, 0, 1); // sets PCA9685 pin HIGH 5V, init Wake, after 1ms should be in Standby
        state = 1;
      } else if (state == 1){
        Serial << "\r\nAUX is in Standby mode, waiting for reset, sending reset pulse";
        outputs.setPin(14, 237, 1);  // Sleep reset pulse
        state = 2;
      } else if (state ==2) {
        Serial << "\r\nAUX is in Active mode, issuing Sleep signal";
        outputs.setPin(14, 0, 0); // sets PCA9685 pin LOW 0V, Deep Sleep
        state = 0;
      }
      uint32_t t2 = micros();
      Serial << "\r\nAUX " << t2 - t1 << "uS";
    }

    else if (usbRead == '5')      // drv9243 testing, Sleep all DRVs (no LEDs)
    {
      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPinAssignments[drvNum], 0, 0);
      }
    }

    else if (usbRead == '6')      // drv9243 testing, Standby all DRVs (red LEDs on LOCK & AUX)
    {
      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPinAssignments[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V, initiate wake-up -> Standby state
      }
    }

    else if (usbRead == '7')      // drv9243 testing, Active all DRVs, AUX green LED, the others white LED if output is active
    {
      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPinAssignments[drvNum], 187, 1);  // Sleep reset pulse
      }
    }

    else if (usbRead == '8')      // drv9243, sleep, wake, activate all DRVs
    {
      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPinAssignments[drvNum], 0, 0); // sets PCA9685 pin LOW 0V, put DRVs to sleep
      }
      delayMicroseconds(150);  // wait max tSLEEP (120uS) for Sleep mode

      // this isn't necessary
      /*for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPinAssignments[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V, initiate wake-up -> Standby state
      }
      delayMicroseconds(1000);  // wait tREADY (1000uS) for Standby
      */

      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPinAssignments[drvNum], 187, 1); // LOW pulse, 187/4096 is 30uS at 1532hz, send nSLEEP reset pulse
      }
      delayMicroseconds(1000);  // wait tREADY (1000uS) for Standby
      // doesn't seem necessary to wait 500uS to set all nSLEEP lines HIGH, just leave them pulsing the Reset pulse
      // This follow setPin isn't needed then either
      /*for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPinAssignments[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V, initiate wake-up -> Standby state
      }*/


      /*(for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPinAssignments[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V, hold nSLEEP HIGh to maintain Active state
      }*/
    }

    else if (usbRead == '9')      // drv9243 searching
    {
      Wire.beginTransmission(0x44);
      Serial.print("\r\n  - Section DRV8243 ");
      if (Wire.endTransmission() == 0)
        Serial.print("found");
      else
        Serial.print("*NOT found!*");

      Wire.beginTransmission(0x70);
      Serial.print("\r\n  - RGB DRV8243 ");
      if (Wire.endTransmission() == 0)
        Serial.print("found");
      else
        Serial.print("*NOT found!*");
    }

    else if (usbRead == '0')      // Sections drv9243 reset
    {
      //outputs.reset();
    }
    else if (usbRead == 13 || usbRead == 10)      // ignore CR or LF
    {
      // do nothing
    }
    #ifdef OGX_H
      else if (usbRead == 'o' && Serial.available() > 0)    // set OGX debug level
      {
        usbRead = Serial.read();
        if (usbRead >= '0' && usbRead <= '5') {
          grade.debugLevel = usbRead - '0';   // convert ASCII numerical char to byte
        }
        Serial.print((String)"\r\nOGX debugLevel: " + grade.debugLevel);
      }
    #endif
    else
    {
      // USB serial data not recognized
      Serial << "\r\n\n*** Unrecognized USB serial input: "; Serial.write(usbRead); Serial << " #" << usbRead << "";
      while (Serial.available()) Serial.read();
    }
  }
}

void printTelem()
{
  if (printStats || !gps1Stats.startupReset)
  {
    if (!gps1Stats.startupReset) {
      ggaMissed = 0;
      ubxParser.relMissed = 0;
    }
    gps1Stats.printStatsReport((char*)"GPS1");
    gps2Stats.printStatsReport((char*)"GPS2");
    relJitterStats.printStatsReport((char*)"RELj");
    relTtrStats.printStatsReport((char*)"RELr");
    bnoStats.printStatsReport((char*)"BNO");
    Serial.println();
  }

  if (printCpuUsages)
  {
    // just hammering testCounter++ in the main loop uses some CPU time
    // baselineProcUsage gets that value, which is used to offset the other usage checks that are hammered in the main loop (or at the same freq)
    uint32_t baselineProcUsage = LOOPusage.reportAve();
    uint32_t dacReport = DACusage.reportAve();          // subracted from AS cpu usage below

    Serial.print("\r\n\nLoop   cpu: "); printCpuPercent(baselineProcUsage);
    Serial.print(" "); Serial.print(testCounter / bufferStatsTimer); Serial.print("kHz"); // up to 400k hits/s

    Serial.print("\r\nBNO_R  cpu: "); printCpuPercent(cpuUsageArray[0]->reportAve(baselineProcUsage));
    Serial.print("\r\nGPS1   cpu: "); printCpuPercent(GPS1usage.reportAve(baselineProcUsage));
    Serial.print("\r\nGPS2   cpu: "); printCpuPercent(GPS2usage.reportAve(baselineProcUsage));
    Serial.print("\r\nPGN    cpu: "); printCpuPercent(PGNusage.reportAve());              // uses a timed update, virtually no extra time penalty
    Serial.print("\r\nAS     cpu: "); printCpuPercent(ASusage.reportAve() - dacReport);   // dac update loop is inside AS update loop (don't want to double count CPU time)
    Serial.print("\r\nNTRIP  cpu: "); printCpuPercent(NTRIPusage.reportAve());            // uses a timed update, virtually no extra time penalty
    Serial.print("\r\nIMU_H  cpu: "); printCpuPercent(IMU_Husage.reportAve());
    Serial.print("\r\nNMEA_P cpu: "); printCpuPercent(NMEA_Pusage.reportAve());
    Serial.print("\r\nUBX_P  cpu: "); printCpuPercent(UBX_Pusage.reportAve());
    Serial.print("\r\nUDP_S  cpu: "); printCpuPercent(UDP_Susage.reportAve());
    Serial.print("\r\nLEDS   cpu: "); printCpuPercent(LEDSusage.reportAve(baselineProcUsage));
    Serial.print("\r\nMach   cpu: "); printCpuPercent(MACHusage.reportAve(baselineProcUsage));
    Serial.print("\r\nESP32  cpu: "); printCpuPercent(ESP32usage.reportAve(baselineProcUsage));
    
    #ifdef AIOv5
      Serial.print("\r\nRS232  cpu: "); printCpuPercent(RS232usage.reportAve(baselineProcUsage));
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
