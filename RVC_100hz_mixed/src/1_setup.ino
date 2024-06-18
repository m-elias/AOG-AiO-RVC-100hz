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
  SerialGPS->begin(baudGPS);
  //SerialGPS->addMemoryForRead(GPSrxbuffer, buffer_size);
  //SerialGPS->addMemoryForWrite(GPStxbuffer, buffer_size);

  //delay(10);
  SerialRTK.begin(baudRTK);
  //SerialRTK.addMemoryForRead(RTKrxbuffer, buffer_size);

  //delay(10);
  SerialGPS2->begin(baudGPS);
  //SerialGPS2->addMemoryForRead(GPS2rxbuffer, buffer_size);
  //SerialGPS2->addMemoryForWrite(GPS2txbuffer, buffer_size);

  #ifdef AIOv50a
    SerialRS232->begin(baudRS232);
    SerialRS232->addMemoryForWrite(RS232txbuffer, buffer_size);
    //SerialRS232->addMemoryForRead(RS232rxbuffer, buffer_size);    // not needed unless custom rs232 rx code is added

    SerialESP32->begin(baudESP32);
    SerialESP32->addMemoryForRead(ESP32rxbuffer, buffer_size);
    SerialESP32->addMemoryForWrite(ESP32txbuffer, buffer_size);
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
  //SerialGPS->clear();
  //SerialGPS2->clear();
  #ifdef AIOv50a
  SerialESP32->clear();
  #endif
  if (BNO.isActive) while (!BNO.read(true));
  machine.watchdogTimer = 0;
  startup = true;
}