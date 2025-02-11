void setCpuFrequency(uint32_t _freq)
{
  set_arm_clock(_freq);
  Serial.printf("\r\nCPU speed set to: %i MHz\r\n", F_CPU_ACTUAL / 1000000);
	delay(10);  // ?
}

void serialSetup()
{
  #ifdef AIOv5
    pinMode(PIEZO_PIN, OUTPUT);
    digitalWrite(PIEZO_PIN, LOW); // piezo/buzzer off
  #endif
  #ifdef AIOv50a
    pinMode(PIEZO2_PIN, OUTPUT);
    digitalWrite(PIEZO2_PIN, LOW);
  #endif

  // setup GPS serial ports here
  SerialGPS1.begin(baudGPS);
  GPS1BAUD = baudGPS;
  SerialGPS1.addMemoryForRead(GPS1rxbuffer, sizeof(GPS1rxbuffer));
  SerialGPS1.addMemoryForWrite(GPS1txbuffer, sizeof(GPS1txbuffer));

  SerialRTK.begin(baudRTK);
  SerialRTK.addMemoryForRead(RTKrxbuffer, sizeof(RTKrxbuffer));

  SerialGPS2.begin(baudGPS);
  GPS2BAUD = baudGPS;
  SerialGPS2.addMemoryForRead(GPS2rxbuffer, sizeof(GPS2rxbuffer));
  SerialGPS2.addMemoryForWrite(GPS2txbuffer, sizeof(GPS2txbuffer));

  #ifdef AIOv5
    SerialRS232.begin(baudRS232);
    //SerialRS232.addMemoryForRead(RS232rxbuffer, sizeof(RS232rxbuffer));    // not needed unless custom rs232 rx code is added
    SerialRS232.addMemoryForWrite(RS232txbuffer, sizeof(RS232txbuffer));

    SerialESP32.begin(baudESP32);
    SerialESP32.addMemoryForRead(ESP32rxbuffer, sizeof(ESP32rxbuffer));
    SerialESP32.addMemoryForWrite(ESP32txbuffer, sizeof(ESP32txbuffer));
  #endif
}

void parserSetup()
{
  // the dash means wildcard
  nmeaParser.setErrorHandler(errorHandler);
  nmeaParser.addHandler("G-GGA", GGA_Handler);
  nmeaParser.addHandler("G-GNS", GNS_Handler);
  nmeaParser.addHandler("G-VTG", VTG_Handler);
  nmeaParser.addHandler("G-HPR", HPR_Handler);
  //const char pvtID[5] = {181, 98, 1, 7, 92};
  //nmeaParser.addHandler(pvtID, PVT_Handler);
  //const char relID = {181, 98, 1, 60, 64};
}

void resetStartingTimersBuffers()
{
  if (BNO.isActive) while (!BNO.read(true));
  SerialGPS1.clear();
  SerialGPS2.clear();
  #ifdef AIOv5
    SerialESP32.clear();
  #endif
  machine.watchdogTimer = 0;
  imuPandaSyncTimer = 0;
  startup = true;
}

#ifdef OGX_H
  void OGX_Setup(){
    #ifdef JD_DAC_H
      grade.setOutput1Handler(updateDacChannel4Output);
    #else
      grade.setOutput1Handler(updatePwmOutput);
    #endif

    grade.setNtripDataHandler(forwardNtripData);
    //grade.setUdpReplyHandler(OgxPgnReplies);
    grade.init(90, 0, 0);    // CAN2RX LED, LOW/0 is ON
  }
#endif
