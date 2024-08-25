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
    else if (usbRead == 'a') // output WAS adc data
    {
        adcDebug = !adcDebug;
        Serial.print("\r\nSetting WAS ADC value debug: ");
        Serial.print(adcDebug);
    }
    else if (usbRead == 'e') // output ESP32 data
    {
        ESP32Debug = !ESP32Debug;
        Serial.print("\r\nSetting ESP32 debug: ");
        Serial.print(ESP32Debug);
    }
    else if (usbRead == 'k') // output Keya data
    {
        KeyaDebug = !KeyaDebug;
        Serial.print("\r\nSetting Keya debug: ");
        Serial.print(KeyaDebug);
    }
    else if (usbRead == 'w') // output workswitch analog data
    {
        workDebug = !workDebug;
        Serial.print("\r\nSetting workswitch debug: ");
        Serial.print(workDebug);
    }
    else if (usbRead == 'n') // output realtime GPS update data
    {
      nmeaDebug = !nmeaDebug;
      ubxParser.debug = nmeaDebug;
      Serial.print("\r\nSetting NMEA debug: ");
      Serial.print(nmeaDebug);
    }
    else if (usbRead == 'p') // output PWM and current sensor data
    {
        pwmDebug = !pwmDebug;
        Serial.print("\r\nSetting PWM / current sensor debug: ");
        Serial.print(pwmDebug);
    }
    else if (usbRead == 'c') // output cpu usage stats
    {
      printCpuUsages = !printCpuUsages;
      Serial.print("\r\nSetting CPU usage debug: ");
      Serial.print(printCpuUsages);
    }
    else if (usbRead == 's') // output GPS, BNO update freq & buffer stats
    {
      printStats = !printStats;
      Serial.print("\r\nSetting Print Stats: ");
      Serial.print(printStats);
    }
#ifdef AIOv50a
    else if (usbRead == 'm' && Serial.available() > 0) // set machine debug level
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5')
      {
        machine.debugLevel = usbRead - '0'; // convert ASCII numerical char to byte
      }
      Serial.print((String) "\r\nMachine debugLevel: " + machine.debugLevel);
    }
#endif
    else if (usbRead == 'g' && Serial.available() > 0) // temporarily set GPS fix state according to standard GGA fix numbers (see LEDS.h, setGpsLED())
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5')
      {
        LEDs.setGpsLED(usbRead - '0', true);
      }
    }
    else if (usbRead == 'l' && Serial.available() > 0) // set RGB brightness
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5')
      {
        LEDs.setBrightness((usbRead - '0') * 50);
        Serial.print("\r\nSetting RGB brightness: ");
        Serial.print((usbRead - '0') * 50);
      }
    }
  }
}

void printTelem()
{
  if (printStats || !gps1Stats.startupReset)
  {
    if (!gps1Stats.startupReset)
    {
      ggaMissed = 0;
      ubxParser.relMissed = 0;
    }
    gps1Stats.printStatsReport((char *)"GPS1");
    gps2Stats.printStatsReport((char *)"GPS2");
    relJitterStats.printStatsReport((char *)"RELj");
    relTtrStats.printStatsReport((char *)"RELr");
    bnoStats.printStatsReport((char *)"BNO");
    Serial.println();
  }

  if (printCpuUsages)
  {
    // just hammering testCounter++ in the main loop uses some CPU time
    // baselineProcUsage gets that value, which is used to offset the other usage checks that are hammered in the main loop (or at the same freq)
    uint32_t baselineProcUsage = LOOPusage.reportAve();
    uint32_t dacReport = DACusage.reportAve(); // subracted from AS cpu usage below

    Serial.print("\r\n\nLoop   cpu: ");
    printCpuPercent(baselineProcUsage);
    Serial.print(" ");
    Serial.print(testCounter / bufferStatsTimer);
    Serial.print("kHz"); // up to 400k hits/s

    Serial.print("\r\nBNO_R  cpu: ");
    printCpuPercent(cpuUsageArray[0]->reportAve(baselineProcUsage));
    Serial.print("\r\nGPS1   cpu: ");
    printCpuPercent(GPS1usage.reportAve(baselineProcUsage));
    Serial.print("\r\nGPS2   cpu: ");
    printCpuPercent(GPS2usage.reportAve(baselineProcUsage));
    Serial.print("\r\nPGN    cpu: ");
    printCpuPercent(PGNusage.reportAve()); // uses a timed update, virtually no extra time penalty
    Serial.print("\r\nAS     cpu: ");
    printCpuPercent(ASusage.reportAve() - dacReport); // dac update loop is inside AS update loop (don't want to double count CPU time)
    Serial.print("\r\nNTRIP  cpu: ");
    printCpuPercent(NTRIPusage.reportAve()); // uses a timed update, virtually no extra time penalty
    Serial.print("\r\nIMU_H  cpu: ");
    printCpuPercent(IMU_Husage.reportAve());
    Serial.print("\r\nNMEA_P cpu: ");
    printCpuPercent(NMEA_Pusage.reportAve());
    Serial.print("\r\nUBX_P  cpu: ");
    printCpuPercent(UBX_Pusage.reportAve());
    Serial.print("\r\nUDP_S  cpu: ");
    printCpuPercent(UDP_Susage.reportAve());
    Serial.print("\r\nLEDS   cpu: ");
    printCpuPercent(LEDSusage.reportAve(baselineProcUsage));
    Serial.print("\r\nMach   cpu: ");
    printCpuPercent(MACHusage.reportAve(baselineProcUsage));
    Serial.print("\r\nESP32  cpu: ");
    printCpuPercent(ESP32usage.reportAve(baselineProcUsage));

#ifdef AIOv50a
    Serial.print("\r\nRS232  cpu: ");
    printCpuPercent(RS232usage.reportAve(baselineProcUsage));
#endif

#ifdef JD_DAC_H
    Serial.print("\r\nDAC    cpu: ");
    printCpuPercent(dacReport);
#endif

    Serial.println();
  }

  testCounter = 0;
  bufferStatsTimer = 0;
}

void printCpuPercent(uint32_t _time)
{
  Serial.printf("%4.1f", (float)_time / 10000.0);
  Serial.print("%");
}
