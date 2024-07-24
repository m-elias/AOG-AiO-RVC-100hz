elapsedMillis oneHertzUpdateTimer;

void oneHertzUpdate()
{
  if (oneHertzUpdateTimer > 999) {           // update loop every 1000ms (1hz)
    oneHertzUpdateTimer = 0;

    // read flow sensor
    // - connected to kickout D (encoder) input
    uint8_t flowRead = encoder.readCount();   // read encoder value for single input (pulsing flow meter)
    //if (flowRead > 0)
      Serial << "\r\nflowRead:" << flowRead;// << " " << encoder.readPosition();

    // send data to AOG with custom PGN?


    encoder.write(0); // clear encoder count
  }
}