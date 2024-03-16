#include "elapsedMillis.h"
#include <stdint.h>
/*
  This is a library written for the Teensy 4.1 microcontroller

  Written by Matt Elias, 2024

  This class is for handling the AIO firmware's LED functionality
  It takes care of translating the LED states for v4.x or v5.0a accordingly
*/


/*
  AgIO Hello received (PWR_ETH RGB) times out if no new Hello received in 5 seconds
    - drops down to ETH_READY (GREEN_BINK)
  GPS fix also times out to stage 0 (OFF) after 1 second of no new GPS fix updates
  
*/

#ifndef H_LED_H
#define H_LED_H

#ifdef AIOv50a
  #include "RGB.h"
#endif

class AIO_LEDS {
private:
  #ifdef AIOv50a
    RGB RGB_LEDS = RGB(1000, 255, 64, 127);   // 1hz RGB update, 255/64/127 RGB brightness balance levels for v5.0a
  #endif
  
  #ifdef AIOv4x
    #define GGA_LED            13  // Teensy onboard LED
    #define PWR_ETH_RED_LED     5  // Red
    #define PWR_ETH_GRN_LED     6  // Green
    #define GPS_RED_LED         9  // Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
    #define GPS_GRN_LED        10  // Green (Flashing = Dual bad, ON = Dual good)
    #define AUTOSTEER_RED_LED  11  // Red
    #define AUTOSTEER_GRN_LED  12  // Green
  #endif

  elapsedMillis agioHelloTimeoutTimer, gpsUpdateTimeoutTimer;
  bool DEBUG = 0;

public:
  AIO_LEDS()
  {
    //asdf
  }
  ~AIO_LEDS(void) {}                      //destructor

  typedef enum {
    PWR_OFF,            // stage 0: RGB OFF
    PWR_ON,             // stage 1: red solid
    NO_ETH,             // stage 2: red blinking
    ETH_READY,          // stage 3: green blinking
    AGIO_CONNECTED,     // stage 4: green solid
  } PWR_ETH_STATES;

  typedef enum {
    ZERO,               // stage 0: RGB OFF
    WAS_ERROR,          // stage 1: red solid
    WAS_READY,          // stage 2: red blinking
    AUTOSTEER_READY,    // stage 3: green blinking
    AUTOSTEER_ACTIVE    // stage 4: green solid
  } STEER_STATES;

#ifdef AIOv50a
  void init() {
    //
  }
  void setSteerLED(STEER_STATES _steerState) {
    //if (_steerState != RGB_LEDS.data[RGB::LED_ID::STEER].stage)
    RGB_LEDS.set(RGB::LED_ID::STEER, _steerState, DEBUG);
  }

  void setPwrEthLED(PWR_ETH_STATES _pwrState) {
    RGB_LEDS.set(RGB::LED_ID::PWR_ETH, _pwrState, DEBUG);
    if (_pwrState == AGIO_CONNECTED) agioHelloTimeoutTimer = 0;
  }

  void setGpsLED(uint8_t _gpsFix, bool _debug = false) {
    //if (_gpsFix != RGB_LEDS.data[RGB::LED_ID::GPS].stage)
    RGB_LEDS.setGpsLED(_gpsFix, _debug);
    gpsUpdateTimeoutTimer = 0;
  }

  void rtcmReceived() {
    RGB_LEDS.queueBlueFlash(RGB::LED_ID::GPS);
  }

  void steerInputAction() {
    RGB_LEDS.activateBlueFlash(RGB::LED_ID::STEER);
  }

  void updateLoop(bool _linkStatus){
    //if (agioHelloTimeoutTimer > 5000 && RGB_LEDS.data[RGB::LED_ID::PWR_ETH].stage != PWR_ETH_STATES::ETH_READY) setPwrEthLED(ETH_READY);
    if (agioHelloTimeoutTimer > 5000) setPwrEthLED(ETH_READY);   // sets PWR__ETH RGB to green_blink if AgIO Hello times out after 5s
    if (gpsUpdateTimeoutTimer > 3000) setGpsLED(3);              // sets GPS RGB OFF if no GPS update for 3s

    RGB_LEDS.updateLoop();
  }
#endif

#ifdef AIOv4x
  void init() {
    pinMode(GGA_LED, OUTPUT);
    pinMode(PWR_ETH_RED_LED, OUTPUT);
    pinMode(PWR_ETH_GRN_LED, OUTPUT);
    pinMode(GPS_RED_LED, OUTPUT);
    pinMode(GPS_GRN_LED, OUTPUT);
    pinMode(AUTOSTEER_GRN_LED, OUTPUT);
    pinMode(AUTOSTEER_RED_LED, OUTPUT);
  }

  void setSteerLED(STEER_STATES _state) {
    if (_state == AUTOSTEER_READY) {    // Autosteer Led goes Red if ADS1115 is found
      digitalWrite(AUTOSTEER_RED_LED, 1);
      digitalWrite(AUTOSTEER_GRN_LED, 0);
    }
    else if (_state == AUTOSTEER_ACTIVE) {
      digitalWrite(AUTOSTEER_RED_LED, 0);
      digitalWrite(AUTOSTEER_GRN_LED, 1);
    }
  }

  void setPwrEthLED(PWR_ETH_STATES _state) {
    /*if (_state == PWR_ON) {
      digitalWrite(PWR_ETH_RED_LED, 1);
      digitalWrite(PWR_ETH_GRN_LED, 0);
    }
    else if (_state == AGIO_CONNECTED) {
      digitalWrite(AUTOSTEER_RED_LED, 0);
      digitalWrite(AUTOSTEER_GRN_LED, 1);
    }*/
    if (_state == AGIO_CONNECTED) agioHelloTimeoutTimer = 0;
  }

  void setGpsLED(uint8_t _gpsFix) {
    gpsUpdateTimeoutTimer = 0;
  }

  void rtcmReceived() {
    // nothing for v4
  }

  void steerInputAction() {
    // nothing for v4
  }

  void updateLoop(bool _linkStatus){
    if (_linkStatus ==  1) {                  //LinkON
      digitalWrite(PWR_ETH_RED_LED, 0);
      digitalWrite(PWR_ETH_GRN_LED, 1);
    } else {
      digitalWrite(PWR_ETH_RED_LED, 1);
      digitalWrite(PWR_ETH_GRN_LED, 0);
    }

    if (gpsUpdateTimeoutTimer > 3000) {       // sets GPS LEDs OFF if no GPS update for 3s
      digitalWrite(GPS_RED_LED, 0);
      digitalWrite(GPS_GRN_LED, 0);
    }

  }
#endif

};
#endif

