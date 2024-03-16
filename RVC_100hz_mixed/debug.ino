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
    #ifdef AIOv50a
    else if (usbRead == 'm' && Serial.available() > 0)
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5') {
        machine.debugLevel = usbRead - '0';   // convert ASCII numerical char to byte
      }
      Serial.print((String)"\r\nMachine debugLevel: " + machine.debugLevel);
    }
    #endif
    else if (usbRead == 'g' && Serial.available() > 0)
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5') {
        LEDs.setGpsLED(usbRead - '0', true);
      }
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
    uint32_t baselineProcUsage = LOOPusage.reportAve();
    uint32_t dacReport = DACusage.reportAve();
    Serial.print("\r\n\nLoop   cpu: "); printCpuPercent(baselineProcUsage);
    Serial.print(" "); Serial.print(testCounter / bufferStatsTimer); Serial.print("kHz"); // up to 400k hits/s
    Serial.print("\r\nBNO_R  cpu: "); printCpuPercent(cpuUsageArray[0]->reportAve(baselineProcUsage));
    Serial.print("\r\nGPS1   cpu: "); printCpuPercent(GPS1usage.reportAve(baselineProcUsage));
    Serial.print("\r\nGPS2   cpu: "); printCpuPercent(GPS2usage.reportAve(baselineProcUsage));
    //Serial.print("\r\nRadio  cpu: "); printCpuPercent(RTKusage.reportAve(baselineProcUsage));
    Serial.print("\r\nPGN    cpu: "); printCpuPercent(PGNusage.reportAve());   // uses a timed update, virtually no extra time penalty
    Serial.print("\r\nAS     cpu: "); printCpuPercent(ASusage.reportAve() - dacReport);
    Serial.print("\r\nNTRIP  cpu: "); printCpuPercent(NTRIPusage.reportAve());  // uses a timed update, virtually no extra time penalty
    Serial.print("\r\nIMU_H  cpu: "); printCpuPercent(IMU_Husage.reportAve());
    Serial.print("\r\nNMEA_P cpu: "); printCpuPercent(NMEA_Pusage.reportAve());
    Serial.print("\r\nUBX_P  cpu: "); printCpuPercent(UBX_Pusage.reportAve());
    Serial.print("\r\nUDP_S  cpu: "); printCpuPercent(UDP_Susage.reportAve());
    Serial.print("\r\nLEDS   cpu: "); printCpuPercent(LEDSusage.reportAve(baselineProcUsage));
    Serial.print("\r\nMach   cpu: "); printCpuPercent(MACHusage.reportAve(baselineProcUsage));
    
    #ifdef AIOv50a
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
