//#include "core_pins.h"
//#include <stdint.h>

#ifndef MISC_H
#define MISC_H

class HighLowHzStats {
public:
  bool isActive;
  bool startupReset;

  HighLowHzStats(void) {}
  ~HighLowHzStats(void) {}

  void printStatsReport(char* _name)
  {
    if (!startupReset) {
      resetLongPeriod();
      resetShortPeriod();
      startupReset = true;
      return;
    }

    if (isActive)
    {
      if (highNumShort > highNumLong) highNumLong = highNumShort;
      if (lowNumShort < lowNumLong) lowNumLong = lowNumShort;
      hzCountLong += hzCountShort;

      uint32_t timeShort = millis() - timerShort;
      uint32_t timeLong = millis() - timerLong;

      Serial.print("\r\n"); Serial.print(_name); 
      Serial.printf("\thi: %i(%i)", highNumShort, highNumLong);
      Serial.printf("\tlo: %i(%i)", lowNumShort, lowNumLong);
      //if (hzCountShort > 0)
        Serial.printf("\tHz: %4.1f(%4.1f)", (float)hzCountShort / ((float)timeShort / 1000.0), (float)hzCountLong / ((float)timeLong / 1000.0));

      Serial.print("   "); Serial.print(timeShort / 1000); Serial.print("s ("); 
      
      if (timeLong > 3600000) {
        Serial.print((float)timeLong / 3600000.0, 1);
        Serial.print("h");
      } else if (timeLong > 60000) {
        Serial.print((float)timeLong / 60000.0, 1);
        Serial.print("m");
      } else {
        Serial.print(timeLong/1000);
        Serial.print("s");
      }
      Serial.print(")");
    }
    else
    {
      resetLongPeriod();
    }

    resetShortPeriod();
  }

  void update(uint16_t _num) {
    isActive = true;
    if (_num > highNumShort) highNumShort = _num;
    if (_num < lowNumShort) lowNumShort = _num;
  }

  void resetShortPeriod() {
    highNumShort = 0;
    lowNumShort = 9999;
    hzCountShort = 0;
    isActive = false;
    timerShort = millis();
  }

  void resetLongPeriod() {
    highNumLong = 0;
    lowNumLong = 9999;
    hzCountLong = 0;
    timerLong = millis();
  }

  void resetAll() {
    resetShortPeriod();
    resetLongPeriod();
  }

  void incHzCount(uint8_t _inc = 1) {
    isActive = true;
    hzCountShort += _inc;
  }

private:
  int32_t hzCountShort, hzCountLong, lowNumShort = 9999, highNumShort, lowNumLong = 9999, highNumLong;
  uint32_t timerShort, timerLong;

};




/*
  This is a class written for the Teensy 4.1 microcontroller
    written by Matt Elias Mar 2024

  This class attempts to profile the Teensy's processor usage by timing how long it takes to process a "task"
  
  There are some considerations:
    1. A microcontroller is always at 100% load (sometimes doing a lot of "nothing"), unless it's sleeping or unpowered.
    2. Even thought a function() may do nothing, calling it repeatedly (hammering it) will build up CPU time in that function
        ie. if it was the only function in loop() then the CPU would spent "100%" of it's time in that "empty" function
    3. Further to #2, all tasks that are "checked" in the main loop incur extra "time" because they are called so frequently
        use a dummy ProcessorUsage object in the main loop to see how much extra time is incurred
    4. Further to #2, "reducing" CPU load on a particular task increases CPU time in other tasks
    5. Using timeIn in a function means you can't use "return;" otherwise timeOut won't be called

*/

/*
  call timeIn() at begining of "task"

  call timeOut() at end of "task"

  reportAve() returns the 1 second average time spent on "task" in uS (\10000 for %)
*/
class ProcessorUsage {
public:
  ProcessorUsage(char* _name) {
    name = _name;
  }
  ~ProcessorUsage(void) {}  // destructor

  void timeIn(uint32_t _time = micros()) {
    periodStartTime = _time;
    isActive = true;
  }

  void timeOut(uint32_t _time = micros()) {
    if (isActive) accumulatedTime += _time - periodStartTime;
    isActive = false;
  }

  uint32_t reportAve(uint32_t _offset = 0) {
    if (isActive) timeOut();  // not sure this should be here. How to deal with an active usage timer when report is called?
    uint32_t t = millis();
    uint32_t oneSecAve = accumulatedTime / ((t - reportStartTime) / 1000) - _offset;
    reset(t);
    return oneSecAve;
  }

  void reset(uint32_t _time = millis()) {
    reportStartTime = _time;
    accumulatedTime = 0;
  }

private:
  uint32_t accumulatedTime, periodStartTime, reportStartTime;
  bool isActive;
  char* name;
};








/*
  Written by Matt Elias 2023

  SpeedPulse is used to calc a frequency for tone()
  according to ISO 11786 standard
*/
class SpeedPulse {
private:
  int8_t outputPin, ledPin;
  elapsedMillis speedTimeoutTimer, pulseUpdateTimer;
  float speed;

public:
  SpeedPulse(int8_t _output_pin, int8_t _led_pin = -1) {
    outputPin = _output_pin;
    ledPin = _led_pin;
    pinMode(outputPin, OUTPUT);
    if (ledPin > -1) pinMode(ledPin, OUTPUT);
  }
  ~SpeedPulse(void) {}  //destructor

  void update() {
    if (speedTimeoutTimer < 1000) {
      if (pulseUpdateTimer > 200)  // 100 (10hz) seems to cause tone lock ups occasionally
      {
        pulseUpdateTimer = 0;

        //130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
        float speedPulse = speed * 36.1111;
        //Serial.println(speedPulse);

        if (speedPulse > 2.0) {  // less then 2 hz (0.055 km/hr) doesn't work well with tone()
          tone(outputPin, uint16_t(speedPulse));
        } else {
          noTone(outputPin);
        }

        if (ledPin > -1) {
          //speedPulse /= 10;
          speedPulse = map(speed, 0.0, 15.0, 1.8, 40.0);  // map 0-15 km/hr to 1.8-40hz
          //Serial << "\r\nspeed:" << speed << " speedPulse:" << speedPulse;
          if (speedPulse > 2.000) {  // less then 2 hz (0.55 km/hr) doesn't work well with tone()
            tone(ledPin, uint16_t(speedPulse));
          } else {
            noTone(ledPin);
          }
        }
      }
    } else {  // if speedTimeoutTimer hasn't updated in 1000 ms, turn off speed pulse
      noTone(outputPin);
      if (ledPin > -1) noTone(ledPin);
    }
  }

  void updateSpeed(float _speed) {
    speed = _speed;
    speedTimeoutTimer = 0;
  }
};

#endif