
void LEDRoutine()
{
	//here can go all the winking and blinking at a human pace

  teensyLedToggle();

	if (gpsLostTimer > 10000) //GGA age over 10sec
	{
		//digitalWrite(GPSRED_LED, LOW);
		//digitalWrite(GPSGREEN_LED, LOW);
	}
}

void teensyLedToggle()
{
  digitalWrite(statLED, !digitalRead(statLED));
}

void teensyLedON()
{
  digitalWrite(statLED, HIGH);
}

void teensyLedOFF()
{
  digitalWrite(statLED, LOW);
}