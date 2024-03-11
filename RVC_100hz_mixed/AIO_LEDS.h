#include "elapsedMillis.h"
#include <stdint.h>
/*
  This is a library written for the Teensy 4.1 microcontroller

  Written by Matt Elias, 2024

  This class is for handling the AIO firmware's LED functionality
  It takes care of translating the LED states for v4.x or v5.0a accordingly
*/


/*
  AgIO Hello received should timeout if no new Hello received in X seconds
  GPS fix should start at solid red (stage 1) and also timeout back to stage 1
  
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
  elapsedMillis agioHelloTimeoutTimer;
  #endif
  

  #ifdef AIOv4x
    #define GGAReceivedLED        13  // Teensy onboard LED
    #define Power_on_LED           5  // Red
    #define Ethernet_Active_LED    6  // Green
    #define GPSRED_LED             9  // Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
    #define GPSGREEN_LED          10  // Green (Flashing = Dual bad, ON = Dual good)
    #define AUTOSTEER_STANDBY_LED 11  // Red
    #define AUTOSTEER_ACTIVE_LED  12  // Green
  #endif

  #define DEBUG  0

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
    if (_steerState != RGB_LEDS.data[RGB::LED_ID::STEER].stage)
      RGB_LEDS.set(RGB::LED_ID::STEER, _steerState, DEBUG);
  }

  void setPwrEthLED(PWR_ETH_STATES _pwrState) {
    RGB_LEDS.set(RGB::LED_ID::PWR_ETH, _pwrState, DEBUG);
    if (_pwrState == AGIO_CONNECTED) agioHelloTimeoutTimer = 0;
  }

  void setGpsLED(uint8_t _gpsFix) {
    if (_gpsFix != RGB_LEDS.data[RGB::LED_ID::GPS].stage)
      RGB_LEDS.setGpsLED(_gpsFix, DEBUG);
  }

  void rtcmReceived() {
    RGB_LEDS.queueBlueFlash(RGB::LED_ID::GPS);
  }

  void steerInputAction() {
    RGB_LEDS.activateBlueFlash(RGB::LED_ID::STEER);
  }

  void updateLoop(){
    if (agioHelloTimeoutTimer > 5000 && RGB_LEDS.data[RGB::LED_ID::PWR_ETH].stage != PWR_ETH_STATES::ETH_READY) setPwrEthLED(ETH_READY);
    RGB_LEDS.updateLoop();
  }
#endif

#ifdef AIOv4x
  void init() {
    pinMode(GGAReceivedLED, OUTPUT);
    pinMode(PowerRed_LED, OUTPUT);
    pinMode(EthernetGreen_LED, OUTPUT);
    pinMode(GPSRED_LED, OUTPUT);
    pinMode(GPSGREEN_LED, OUTPUT);
    pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);
    pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
  }
  void setSteerLED(STEER_STATES _steerState) {
    if (_steerState == AUTOSTEER_READY) {    // Autosteer Led goes Red if ADS1115 is found
      digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
      digitalWrite(AUTOSTEER_STANDBY_LED, 1);
    }
    if (_steerState == AUTOSTEER_ACTIVE) {
      // Autosteer Led goes GREEN if autosteering
      //digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
      //digitalWrite(AUTOSTEER_STANDBY_LED, 0);
    }
  }

  void setPwrEthLED(PWR_ETH_STATES _pwrState) {

  }

  void setGpsLED(uint8_t _gpsFix) {

  }

  void rtcmReceived() {
    
  }

  void steerInputAction() {
    
  }

  void updateLoop(){
    if (UDP.linkStatus() == LinkON) {
      digitalWrite(Power_on_LED, 0);
      digitalWrite(Ethernet_Active_LED, 1);
    } else {
      digitalWrite(Power_on_LED, 1);
      digitalWrite(Ethernet_Active_LED, 0);
    }

    //asdf

  }
#endif

};
#endif


  //pinMode(statLED, OUTPUT);

/*

constexpr auto statLED = LED_BUILTIN;   //Teensy onboard LED, p13
*/
/*
void LEDRoutine()
{
	//here can go all the winking and blinking at a human pace

  teensyLedToggle();

	if (gpsLostTimer > 10000) //GGA age over 10sec
	{
		//digitalWrite(GPSRED_LED, LOW);
		//digitalWrite(GPSGREEN_LED, LOW);
	}
  LEDTimer = 0;
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
*/