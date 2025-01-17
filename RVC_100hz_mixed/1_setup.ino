void setCpuFrequency(uint32_t _freq)
{
  set_arm_clock(_freq);
  Serial.printf("\r\n\nCPU speed set to: %i MHz", F_CPU_ACTUAL / 1000000);
	delay(10);  // ?
}

void serialSetup()
{
  #ifdef AIOv50a
    pinMode(PIEZO1, OUTPUT);
    pinMode(PIEZO2, OUTPUT);
    digitalWrite(PIEZO1, LOW);
    digitalWrite(PIEZO2, LOW);
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

  #ifdef AIOv50
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
}

void resetStartingTimersBuffers()
{
  //machine.watchdogTimer = 0;
  if (BNO.isActive) while (!BNO.read(true));
  SerialGPS1.clear();
  SerialGPS2.clear();
  #ifdef AIOv50
  SerialESP32.clear();
  #endif
  machine.watchdogTimer = 0;
  imuPandaSyncTimer = 0;
  startup = true;
}