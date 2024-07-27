elapsedMillis oneHertzUpdateTimer;

void oneHertzUpdate()
{
  if (oneHertzUpdateTimer > 999) {           // update loop every 1000ms (1hz)
    oneHertzUpdateTimer = 0;

  }
}