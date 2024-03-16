void setCpuFrequency(uint32_t _freq)
{
  set_arm_clock(_freq);
  Serial << "\r\n\nCPU speed set to: " << F_CPU_ACTUAL / 1000000 << "MHz";
	delay(10);  // ?
}

void serialSetup()
{
  // setup GPS serial ports here
  SerialGPS->begin(baudGPS);
  SerialGPS->addMemoryForRead(GPSrxbuffer, buffer_size);
  SerialGPS->addMemoryForWrite(GPStxbuffer, buffer_size);

  //delay(10);
  SerialRTK.begin(baudRTK);
  SerialRTK.addMemoryForRead(RTKrxbuffer, buffer_size);

  //delay(10);
  SerialGPS2->begin(baudGPS);
  SerialGPS2->addMemoryForRead(GPS2rxbuffer, buffer_size);
  SerialGPS2->addMemoryForWrite(GPS2txbuffer, buffer_size);

  #ifdef AIOv50a
    SerialRS232->begin(baudRS232);
    SerialRS232->addMemoryForWrite(RS232txbuffer, buffer_size);
    //SerialRS232->addMemoryForRead(RS232rxbuffer, buffer_size);    // not needed unless custom rs232 rx code is added

    SerialESP32->begin(baudESP32);
    SerialESP32->addMemoryForRead(ESP32rxbuffer, buffer_size);
    SerialESP32->addMemoryForWrite(ESP32txbuffer, buffer_size);
  #endif

  #ifdef JD_DAC_H
    jdDac.setDebugStream(&Serial);
  #endif
}

void parserSetup()
{
  // the dash means wildcard
  nmeaParser.setErrorHandler(errorHandler);
  nmeaParser.addHandler("G-GGA", GGA_Handler);
  nmeaParser.addHandler("G-GNS", GNS_Handler);
  nmeaParser.addHandler("G-VTG", VTG_Handler);
}

void resetStartingTimersBuffers()
{
  //machine.watchdogTimer = 0;
  SerialGPS->clear();
  SerialGPS2->clear();
  if (BNO.isActive) while (!BNO.read(true));
  #ifdef AIOv50a
  machine.watchdogTimer = 0;
  #endif
  startup = true;
}