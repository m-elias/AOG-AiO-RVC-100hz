void calcSteeringPID(void)
{
  #ifdef JD_DAC_H
    float pValue = steerSettings.Kp * steerAngleSetPoint; // only use set point, not error for two track JD
    float errorAbs = abs(steerAngleSetPoint);
  #else
    float pValue = steerSettings.Kp * steerAngleError;
    float errorAbs = abs(steerAngleError);
  #endif

  pwmDrive = (int16_t)pValue;
  /*Serial.print("\r\n");
  Serial.print(" ");
  Serial.print(pwmDrive);*/

  //add min throttle factor so no delay from motor resistance.
  if (pwmDrive < 0 ) pwmDrive -= steerSettings.minPWM;
  else if (pwmDrive > 0 ) pwmDrive += steerSettings.minPWM;
  //Serial.print(" ");
  //Serial.print(pwmDrive);

  int16_t newHighPWM = 0;

  // from 0-3 deg error, scale newHighPWM from lowPWM(minPWM*1.2)-highPWM
  if (errorAbs < LOW_HIGH_DEGREES) {  
    newHighPWM = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
  }
  else newHighPWM = steerSettings.highPWM;

  //limit the pwm drive
  // causes oscillation in pwmDrive
  if (pwmDrive > newHighPWM) pwmDrive = newHighPWM;
  if (pwmDrive < -newHighPWM) pwmDrive = -newHighPWM;
  //Serial.print(" ");
  //Serial.print(pwmDrive);

  if (steerConfig.MotorDriveDirection) pwmDrive *= -1;
  //Serial.print(" ");
  //Serial.print(pwmDrive);

  if (steerConfig.IsDanfoss)  {
    // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
    // Danfoss: PWM 50% On = Center Position
    // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
    pwmDrive = (constrain(pwmDrive, -250, 250));

    // Calculations below make sure pwmDrive values are between 65 and 190
    // This means they are always positive, so in motorDrive, no need to check for
    // steerConfig.isDanfoss anymore
    pwmDrive = pwmDrive >> 2; // Divide by 4
    pwmDrive += 128;          // add Center Pos.

  }
}

//#########################################################################################

void motorDrive(void)
{
  if (steerConfig.CytronDriver)
  {
    #ifdef JD_DAC_H
      // For JD_DAC.h, MCP4728 QUAD DAC steering
      // scale pwmDrive to DAC output
      // 0 PWM (no WAS change needed) = 2048 centered DAC output (4096 / 2 to get center voltage)
      DACusage.timeIn();
      //if (gpsSpeed < (float)steerConfig.MinSpeed / 10.0) pwmDrive = 0;
      pwmDisplay = jdDac.steerOutput(pwmDrive);
      jdDac.ch4Output(pwmDrive);
      DACusage.timeOut();
    #else    
      // Cytron Driver Dir + PWM Signal
      if (pwmDrive > 0) {
        digitalWrite(DIR_PIN, HIGH);
      }
      else {
        digitalWrite(DIR_PIN, LOW);
        pwmDrive = -1 * pwmDrive;
      }

      analogWrite(PWM_PIN, pwmDrive);  //write out the 0 to 255 value
      pwmDisplay = pwmDrive;
    #endif
  }
  else {
    // IBT 2 Driver DIR_PIN connected to BOTH enables
    // PWM_PIN Left + SLEEP_PIN Right Signal
    // kept in case someone hacked their AIO to use IBT2 style driver

    if (pwmDrive > 0) {
      analogWrite(SLEEP_PIN, 0);          //Turn off before other one on
      analogWrite(PWM_PIN, pwmDrive);
    }
    else {
      pwmDrive = -1 * pwmDrive;
      analogWrite(PWM_PIN, 0);            //Turn off before other one on
      analogWrite(SLEEP_PIN, pwmDrive);
    }
    pwmDisplay = pwmDrive;
  }
}
