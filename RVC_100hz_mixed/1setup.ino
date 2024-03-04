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

    SerialESP32->begin(baudESP32);
    //SerialESP32->addMemoryForRead(ESP32rxbuffer, buffer_size);
    //SerialESP32->addMemoryForWrite(ESP32txbuffer, buffer_size);
  #endif

  #ifdef JD_DAC_H
    jdDac.setDebugStream(&Serial);
  #endif
}

void ioSetup()
{
  pinMode(statLED, OUTPUT);

  #ifdef AIOv50a
    pinMode(PIEZO1, OUTPUT);
    pinMode(PIEZO2, OUTPUT);
    digitalWrite(PIEZO1, HIGH);
    digitalWrite(PIEZO2, HIGH);
  #endif

  #ifdef AIOv4x
    pinMode(PowerRed_LED, OUTPUT);
    pinMode(EthernetGreen_LED, OUTPUT);
    pinMode(GPSRED_LED, OUTPUT);
    pinMode(GPSGREEN_LED, OUTPUT);
    pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
    pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);
  #endif
  //Serial.print("\r\n- IO set");
}

void parserSetup()
{
  // the dash means wildcard
  nmeaParser.setErrorHandler(errorHandler);
  nmeaParser.addHandler("G-GGA", GGA_Handler);
  nmeaParser.addHandler("G-VTG", VTG_Handler);
}
