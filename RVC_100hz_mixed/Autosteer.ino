/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
const uint8_t PWM_Frequency = 2;
const float LOW_HIGH_DEGREES = 3.0;  // How many degrees before decreasing Max PWM

bool testBothWasSensors = false;
bool adcDebug = false;
bool useInternalADC = false;
bool useExternalADS = false;

#include <EEPROM.h>
const int EE_Ident = 2401;  // if value in eeprom does not match, overwrite with defaults

uint32_t autoSteerLastTime, currentTime;
elapsedMillis autoSteerUpdateTimer;
const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2;  // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
uint8_t aog2Count = 0;
bool autoSteerEnabled = false;
float gpsSpeed;

// Relays
/*bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;*/
uint8_t xte = 0;

// Switches/Sensors
uint8_t kickoutInput = 0, workInput = 0, steerState = 0, switchByte = 0;
float sensorReading, sensorSample;

// On Off
uint8_t guidanceStatus = 0, prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

// steering variables
float steerAngleActual = 0, steerAngleSetPoint = 0, steerAngleError = 0;
int16_t steeringPosition = 0;  // from steering sensor (WAS)

// pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float highLowPerDeg = 0;

// Steer switch button  ***********************************************************************************************************
uint8_t steerReading, prevSteerReading = 1;  // currentState = 0
int16_t pulseCount = 0;                      // Steering Wheel Encoder
int16_t lastEnc = -999;

// Variables for settings
struct SteerSettingsStruct {
  uint8_t Kp = 40;      // proportional gain
  uint8_t lowPWM = 10;  // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 150;  // max PWM value
  float steerSensorCounts = 120;
  float AckermanFix = 1;  // sent as percent
};
SteerSettingsStruct defaultSteerSettings;                         // 11 bytes
struct SteerSettingsStruct steerSettings = defaultSteerSettings;  // don't need 'struct' in front?

// Variables for settings - 0 is false
struct SteerConfigStruct {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0;  // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;  // 1 if switch selected
  uint8_t SteerButton = 0;  // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 3;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0;  // Set to 0 to use X Axis, 1 to use Y avis
  uint8_t MinSpeed = 0;
};
SteerConfigStruct const defaultSteerConfig;  // 9 bytes
struct SteerConfigStruct steerConfig = defaultSteerConfig;

void steerConfigInit() {
  if (steerConfig.CytronDriver) {
    pinMode(SLEEP_PIN, OUTPUT);
    if (steerConfig.SteerButton == 0 && steerConfig.SteerSwitch == 0) {
      // currentState = 0;
      prevSteerReading = 1;
    }
  }

  if (steerConfig.PressureSensor) {
    pinMode(KICKOUT_A_PIN, INPUT_DISABLE);
  } else {
    pinMode(KICKOUT_A_PIN, INPUT_PULLUP);
  }

  BNO.isSwapXY = !steerConfig.IsUseY_Axis;
}

void steerSettingsInit() {
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void autosteerSetup() {
  Serial.print("\r\n\nAutoSteer setup");
  // PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0) {
    analogWriteFrequency(PWM_PIN, 490);
    analogWriteFrequency(SLEEP_PIN, 490);
  } else if (PWM_Frequency == 1) {
    analogWriteFrequency(PWM_PIN, 122);
    analogWriteFrequency(SLEEP_PIN, 122);
  } else if (PWM_Frequency == 2) {
    analogWriteFrequency(PWM_PIN, 3921);
    analogWriteFrequency(SLEEP_PIN, 3921);
  }

  pinMode(DIR_PIN, OUTPUT);

  // keep pulled high and drag low to activate, noise free safe
  pinMode(STEER_PIN, INPUT_PULLUP);
  pinMode(KICKOUT_D_PIN, INPUT_PULLUP);  // also set by Encoder library

// Disable pullup/down resistors for analog input pins
#ifdef AIOv5
  pinMode(WORK_PIN, INPUT_DISABLE);
#else
  pinMode(WORK_PIN, INPUT_PULLUP);
#endif
  pinMode(CURRENT_PIN, INPUT_DISABLE);
  // pinMode(KICKOUT_A_PIN, INPUT_PULLUP);   // set in steerConfigInit() according to function
  // pinMode(KICKOUT_A_PIN, INPUT_DISABLE);

  uint16_t as_ee_read = EE_Ident;
  EEPROM.get(1, as_ee_read);

  if (as_ee_read != EE_Ident) {  // if value in eeprom does not match, overwrite with defaults
    EEPROM.put(1, EE_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    Serial.print("\r\n- ** EEPROM reset to defaults! **");
  } else {
    EEPROM.get(10, steerSettings);  // read the Settings
    EEPROM.get(40, steerConfig);
    Serial.print("\r\n- loaded settings/config from EEPROM");
  }

  steerSettingsInit();
  steerConfigInit();
  adcSetup();

  if (!autoSteerEnabled) {
    Serial.print("\r\n- ** AutoSteer is disabled, GPS only mode **");
    Serial.print("\r\n  - ** likely no WAS input detected **");
    return;
  }

  Serial.print("\r\n- AutoSteer enabled, setup complete");
  LEDs.set(LED_ID::STEER, STEER_STATE::AUTOSTEER_READY);
}  // End of autosteerSetup



void autoSteerUpdate() {
  ASusage.timeIn();

  if (autoSteerUpdateTimer > 9) {  // update AS loop every 10ms (100hz)
    autoSteerUpdateTimer -= 10;    // or = 0?

    // 1 PCB Button pressed?
    uint8_t reading = digitalRead(STEER_PIN);

    if (steerConfig.SteerSwitch == 1) {
      // Switch is off so reset ready for next switch on
      if (reading == HIGH) {
        steerState = 1;
        prevSteerReading = reading;
      }
    }

    // 2 Has tablet button been pressed?
    if (guidanceStatusChanged) {
      if (guidanceStatus == 1) {  //Must have changed Off >> On
        steerState = 0;
      }
    }

    // If AOG has stopped steering, wait then turn off steerswitch ready for next engage.
    static int switchCounter = 0;

    if (steerState == 0 && guidanceStatus == 0) {
      if (switchCounter++ > 30) {
        steerState = 1;
      }
    } else {
      switchCounter = 0;
    }

    // Arduino software button code
    if (reading == LOW && prevSteerReading == HIGH) {
      steerState = !steerState;
    }
    prevSteerReading = reading;


    // ******************************* Steer Switch/Button *******************************
    // Steer input logic all setup so that '1' (HIGH) is ON, and '0' (LOW) is OFF
    /*steerReading = !digitalRead(STEER_PIN);  // read steer input switch/button, invert reading to match On/Off logic
    // steerReading = analogRead(KICKOUT_A_PIN) > ANALOG_TRIG_THRES ? LOW : HIGH;

    if (steerConfig.SteerSwitch == 1)  // steer "Switch" mode (on - off)
    {
      // new code for steer "Switch" mode that keeps AutoSteer OFF after current/pressure kickout until switch is cycled
      if (steerReading == LOW) {  // switching OFF
        steerState = LOW;         // set OFF
        if (prevSteerReading != steerState) {
          //char msg[] = "AutoSteer Switch OFF";
          //char msgTime = 2;
          //UDP.SendUdpFreeForm(1, msg, strlen(msg), msgTime, UDP.broadcastIP, UDP.portAgIO_9999);
          LEDs.activateBlueFlash(LED_ID::STEER);
        }
      } else if (steerReading == HIGH && prevSteerReading == LOW) {  // switch ON after prev being OFF
        steerState = HIGH;                                           // set ON
        //char msg[] = "AutoSteer Switch ON";
        //char msgTime = 2;
        //UDP.SendUdpFreeForm(1, msg, strlen(msg), msgTime, UDP.broadcastIP, UDP.portAgIO_9999);
        LEDs.activateBlueFlash(LED_ID::STEER);
      }
      prevSteerReading = steerReading;
    }

    else if (steerConfig.SteerButton == 1)  // steer "Button" mode (momentary)
    {
      if (steerReading == HIGH && prevSteerReading == LOW) {  // button is pressed
        steerState = !steerState;
        LEDs.activateBlueFlash(LED_ID::STEER);
        //char* msg;
        //if (steerState) msg = (char*)"AutoSteer Btn ON";
        //else msg = (char*)"AutoSteer Btn OFF";
        //char msgTime = 2;
        //UDP.SendUdpFreeForm(1, msg, strlen(msg), msgTime, UDP.broadcastIP, UDP.portAgIO_9999);
      }
      prevSteerReading = steerReading;  // get ready to detect next press

      if (guidanceStatusChanged) steerState = guidanceStatus;  // allows AoG to turn AS on/off in parallel with Btn
    }

    else  // No steer switch or button
    {
      // If steering is OFF and AoG's GUI btn is switched ON
      if (guidanceStatusChanged && guidanceStatus == 1 && steerState == 0 && prevSteerReading == 1) {
        prevSteerReading = steerState;
        steerState = 1;
        // prevSteerReading = !steerState;
        LEDs.activateBlueFlash(LED_ID::STEER);
      }

      // If steering is ON and AoG's GUI btn is switched OFF
      if (guidanceStatusChanged && guidanceStatus == 0 && steerState == 1 && prevSteerReading == 0) {
        prevSteerReading = steerState;
        steerState = 0;
        // prevSteerReading = !steerState;
        LEDs.activateBlueFlash(LED_ID::STEER);
      }
    }*/



    // ******************* Kickouts ( Encoders / Pressure / Current ) *******************
    if (steerConfig.ShaftEncoder) {
      if (encoderType == 1)  // single input
      {
        pulseCount = encoder.readCount();
        if (pulseCount != lastEnc) {
          // Serial << "\r\npulseCount:" << pulseCount << " " << encoder.readPosition();
          lastEnc = pulseCount;
        }
      } else if (encoderType == 2)  // dual input (quadrature encoder)
      {
        pulseCount = abs(encoder.readPosition());
        if (pulseCount != lastEnc) {
          Serial << "\r\npulseCount:" << pulseCount;
          lastEnc = pulseCount;
        }
      }
      if (pulseCount >= steerConfig.PulseCountMax) {
        steerState = 0;  // reset values like it turned off
        prevSteerReading = !steerState;
      }
    }

    // Pressure sensor?
    if (steerConfig.PressureSensor) {
      sensorSample = (float)analogRead(KICKOUT_A_PIN);  // >> 4);    // to scale 12 bit down to 8 bit
      // Serial << "\r\n" << sensorSample;
      sensorSample *= 0.15;  // for 5v sensor, scale down to try matching old AIO
      // sensorSample *= 0.0625;                      // for 12v sensor
      // Serial << " " << sensorSample;

      sensorSample = min(sensorSample, 255);                     // limit to 1 byte (0-255)
      sensorReading = sensorReading * 0.8 + sensorSample * 0.2;  // filter
      // Serial << " " << sensorSample << " max:" << steerConfig.PulseCountMax;

      if (sensorReading >= steerConfig.PulseCountMax) {  // if reading exceeds kickout setpoint
        steerState = 0;                                  // turn OFF autoSteer
        prevSteerReading = !steerState;
      }
    }

    // Current sensor?
    if (steerConfig.CurrentSensor) {
      sensorSample = (float)analogRead(CURRENT_PIN);
      // Serial << "\r\n" << sensorSample;

#ifdef AIOv5
      // sensorSample = abs((sensorSample - ???)) * 0.0625;       // for v5.0a ACS711 (untested), output is not inverted
      sensorSample = abs(sensorSample - 240) * 0.0625;  // for v5.0a DRV8701, output is not inverted
#else
      sensorSample = abs(3100 - sensorSample) * 0.0625;  // 3100 is like old firmware, 3150 is center (zero current) value on Matt's v4.0 Micro
#endif

      // Serial << " " << sensorSample;
      sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
      // Serial << " " << sensorReading << " max:" << steerConfig.PulseCountMax;
      if (sensorReading >= steerConfig.PulseCountMax) {
        steerState = 0;  // turn OFF autoSteer
        prevSteerReading = !steerState;
      }
    }

#ifdef AIOv5
    uint8_t read = analogRead(WORK_PIN) > ANALOG_TRIG_THRES ? HIGH : LOW;  // read work input
#else
    uint8_t read = digitalRead(WORK_PIN);
#endif
    if (read != workInput) {
      Serial.printf("\r\nWORK input: %s", (read == 1 ? "OFF" : "ON"));
      workInput = read;
    }

    switchByte = 0;
    switchByte |= (kickoutInput << 2);  // put remote in bit 2
    switchByte |= (steerState << 1);   // put steerInput status in bit 1 position
    switchByte |= workInput;

    // Serial << " <> " << digitalRead(KICKOUT_D_PIN) << ":" << digitalRead(KICKOUT_A_PIN) << ":" << analogRead(CURRENT_PIN);
    // Serial << "\r\npsr:" << prevSteerReading << " ss:" << steerState << " gs:" << guidanceStatus << " gsc:" << guidanceStatusChanged;


    // ***************************** READ WAS *****************************
    // useExternalADS = true;

#ifndef JD_DAC_H
    if (adcDebug) Serial.printf("\r\n%6i", millis());
    if (useInternalADC || testBothWasSensors) {
      steeringPosition = int(float(teensyADC->adc1->analogRead(WAS_SENSOR_PIN)) * 3.23);
      if (adcDebug) Serial.printf(" Teensy ADC(x3.23):%5i", steeringPosition);
    }
    int16_t temp = steeringPosition;
    if (useExternalADS || testBothWasSensors) {
      steeringPosition = ads1115.getConversion();
      steeringPosition = (steeringPosition >> 1);  // bit shift by 1  0 to 13610 is 0 to 5v
      if (adcDebug) Serial.printf(" ADS1115:%5i", steeringPosition);
    }
    if (testBothWasSensors && adcDebug) {
      Serial.printf("  %.2f", float(steeringPosition) / float(temp));
    }
#else
    DACusage.timeIn();
    jdDac.update();
    // static int16_t oldSteer;
    int16_t newDacSteering = (jdDac.getWAS() >> 1);  // read JD SWS instead to display on AoG
    // if (adcDebug && (newDacSteering > oldSteer +10 || newDacSteering < oldSteer -10)) Serial.printf("\r\n%6i  DAC_ADS-ch0(/2):%5i", millis(), newDacSteering);
    if (adcDebug || (analogRead(WORK_PIN) > ANALOG_TRIG_THRES ? LOW : HIGH)) Serial.printf("\r\n%6i  DAC_ADS-ch0(/2):%5i", millis(), newDacSteering);
    steeringPosition = newDacSteering;
    // oldSteer = steeringPosition;
    DACusage.timeOut();

    // if (!digitalRead(KICKOUT_D_PIN)) jdDac.readAllSWS();
#endif

    // DETERMINE ACTUAL STEERING POSITION
    // convert position to steer angle. 32 counts per degree of steer pot position in my case
    //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if (steerConfig.InvertWAS) {
      steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset);  // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
    } else {
      steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset);  // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
    }

    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);  // Ackerman fix
    steerAngleError = steerAngleActual - steerAngleSetPoint;                                      // calculate the steering error
    // if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;



    // ******************************** WATCHDOG checks & PWM output ********************************
    // if (steerState == 0) {
    // watchdogTimer = WATCHDOG_FORCE_VALUE;  //turn off steering motor
    //}// else {                                 //valid conditions to turn on autosteer
    //watchdogTimer = 0;                     //reset watchdog
    //}

    // If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) {
      watchdogTimer = WATCHDOG_FORCE_VALUE;
      steerState = 1; // reset values like it turned off
    }

    //Serial.print("\r\nAS wd: "); Serial.print(watchdogTimer);
    if (watchdogTimer < WATCHDOG_THRESHOLD) {
      // Enable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver) {
#ifdef JD_DAC_H
        jdDac.steerEnable(true);  // select IBT2 for JD DAC control
#else
        digitalWrite(SLEEP_PIN, steerConfig.IsRelayActiveHigh ? LOW : HIGH);
#endif
      } else {
        digitalWrite(DIR_PIN, 1);
      }

      calcSteeringPID();  //do the pid
      motorDrive();       //out to motors the pwm value

      LEDs.set(LED_ID::STEER, STEER_STATE::AUTOSTEER_ACTIVE);

    } else {
      //we've lost the comm to AgOpenGPS, or just stop request
      //Disable H Bridge for IBT2, hyd aux, etc for cytron
      pwmDrive = 0;  //turn off steering motor
      pulseCount = 0;
      encoder.write(0);

      if (steerConfig.CytronDriver) {
#ifdef JD_DAC_H
        jdDac.steerEnable(false);
        //jdDac.ch4Enable(false);
#else
        digitalWrite(SLEEP_PIN, steerConfig.IsRelayActiveHigh ? bool(!pwmDrive) : bool(pwmDrive));
#endif
      } else {
        digitalWrite(DIR_PIN, 0);  //IBT2
      }

      motorDrive();  //out to motors the pwm value

      LEDs.set(LED_ID::STEER, STEER_STATE::AUTOSTEER_READY);

      //Serial.print("\r\n\n*** Autosteer watchdog triggered - Autosteer disabled! ***\r\n");
    }

    //Serial << " pwm:" << pwmDrive; // << " <> " << (steerConfig.IsRelayActiveHigh ? bool(!pwmDrive) : bool(pwmDrive));

    //Serial.print(", loop run time: "); Serial.print(micros() - autsteerStartTimeuS); Serial.print("uS, ");
  }

  ASusage.timeOut();
}  // end of autoSteerLoop

/*
  adcSetup() detects whether to use Teensy ADC or I2C ADS1115
  - Teensy ADC set to oversample at 12 bits, average 16 readings at medium speed
    - 32uS to read Teensy ADC
  - ADS1115 set to continously sample as fast as possible
    - 485uS to retrieve ADS1115 value via I2C
*/
void adcSetup() {
  ads1115.setWirePort(I2C_WIRE);

  Serial.print("\r\n- ADC check:");
  teensyADC->adc0->setAveraging(16);                                     // set number of averages
  teensyADC->adc0->setResolution(12);                                    // set bits of resolution
  teensyADC->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);  // change the conversion speed
  teensyADC->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);      // change the sampling speed
  teensyADC->adc1->setAveraging(16);
  teensyADC->adc1->setResolution(12);
  teensyADC->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  teensyADC->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);

  // detect input on Teensy WAS_SENSOR_PIN
  pinMode(WAS_SENSOR_PIN, INPUT_PULLDOWN);
  // delay(5);     // with ide/teensyduino update these delays cause Serial.print to fail in the rest of adsSetup()
  uint16_t pullDown = teensyADC->adc1->analogRead(WAS_SENSOR_PIN);
  pinMode(WAS_SENSOR_PIN, INPUT_PULLUP);
  // delay(1);    // previously these delays were needed to allow time for the adc to settle but doesn't seem the case anymore
  uint16_t pullUp = teensyADC->adc1->analogRead(WAS_SENSOR_PIN);
  uint16_t pullDiff = abs(pullUp - pullDown);
  pinMode(WAS_SENSOR_PIN, INPUT_DISABLE);  // don't forget to disable the internal resistor !!
  Serial.printf("\r\n  - A0 pDn:%4i, pUp:%4i, diff:%4i", pullDown, pullUp, pullDiff);
  // Serial.print((String)"\r\n  - A0 pDn:" + pullDown + ", pUp:" + pullUp + ", diff:" + pullDiff);    // same as above

  if (pullDiff < 500)  // v4.0, A0 floating 3960 diff, MCP plugged in 140 diff max
  {
    Serial.print("\r\n  - using Teensy ADC");
    useInternalADC = true;
    autoSteerEnabled = true;
    LEDs.set(LED_ID::STEER, STEER_STATE::WAS_READY);
    if (!testBothWasSensors)
      return;
  }

  if (!testBothWasSensors)
    Serial << "\r\n  - Teensy ADC P" << WAS_SENSOR_PIN << " is unconnected/floating?";
  Serial.print("\r\n  - checking for I2C ADS1115");

  I2C_WIRE.end();
  I2C_WIRE.begin();
  if (ads1115.testConnection()) {
    Serial.print("\n  - ADS1115 found");
    if (!useInternalADC)
      useExternalADS = true;
    autoSteerEnabled = true;
    LEDs.set(LED_ID::STEER, STEER_STATE::WAS_READY);
    ads1115.setSampleRate(ADS1115_REG_CONFIG_DR_860SPS);
    ads1115.setGain(ADS1115_REG_CONFIG_PGA_6_144V);
    ads1115.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  // ************set according to EEPROM (saved from PGN)*****************
    ads1115.triggerConversion(true);                  // to start continous mode
  } else {
    Serial.print("\n\n**** No WAS input detected! ****\n\n");
    autoSteerEnabled = false;
    LEDs.set(LED_ID::STEER, STEER_STATE::WAS_ERROR);
  }
}  // end adcSetup()
