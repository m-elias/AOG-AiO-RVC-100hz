/*
  This is a library written for AgOpenGPS AIO PCBs with RGB faceplate LEDs
  - written by Matt Elias, 2024

  It uses the Adafruit_NeoPixel library to control (4) faceplate facing RGB LEDs via (4) WS2811 chips using NRZ protocol

  The goal of this class is to retrieve states from other classes (rtkStatus, ethStatus, steerStatus, etc)
  and decode those values into RGB color/blink states. That way all the RGB/LED code stays in this class
  instead of strewn about all over the rest of the code. I hope this works! ;)

*/

#ifndef H_LED_H
#define H_LED_H

#include "elapsedMillis.h"
#include "Adafruit_NeoPixel.h"  // https://github.com/adafruit/Adafruit_NeoPixel/blob/master/Adafruit_NeoPixel.h
#include "core_pins.h"
#include <stdint.h>

typedef enum {
  PWR_ETH,
  GPS,
  STEER,
  UNUSED
} LED_ID;

typedef enum {
  STAGE0_OFF,
  STAGE1_RED,
  STAGE2_RED_BLINK,
  STAGE3_GREEN_BLINK,
  STAGE4_GREEN,
} LED_STAGE;

typedef enum {
  PWR_OFF,            // stage 0: LED OFF
  PWR_ON,             // stage 1: red solid
  NO_ETH,             // stage 2: red blinking
  ETH_READY,          // stage 3: green blinking
  AGIO_CONNECTED,     // stage 4: green solid
} PWR_ETH_STATE;

typedef enum {
  ZERO,               // stage 0: LED OFF
  WAS_ERROR,          // stage 1: red solid
  WAS_READY,          // stage 2: red blinking
  AUTOSTEER_READY,    // stage 3: green blinking
  AUTOSTEER_ACTIVE    // stage 4: green solid
} STEER_STATE;

class LEDS {
private:
  #ifdef AIOv4x
    /*#define PWR_ETH_RED_LED   5  // LED 1 - Red
    #define PWR_ETH_GRN_LED     6  // LED 1 - Green
    #define GPS_RED_LED         9  // LED 2 - Red
    #define GPS_GRN_LED        10  // LED 2 - Green
    #define AUTOSTEER_RED_LED  11  // LED 3 - Red
    #define AUTOSTEER_GRN_LED  12  // LED 3 - Green*/
    const uint8_t v4redLEDio[3] = { 5, 9, 11 };
    const uint8_t v4grnLEDio[3] = { 6, 10, 12 };
    const uint8_t NUM_LEDS = 3;
  #else 
    const uint8_t NUM_LEDS = 4;
  #endif

  const uint8_t WS2811_PIN = 33;    // unused IO on v4
  Adafruit_NeoPixel WS2811 = Adafruit_NeoPixel(NUM_LEDS, WS2811_PIN, NEO_BGR + NEO_KHZ800);
  #define GGA_LED    13  // Teensy built-in LED
    
  uint8_t mainBrightness       = 255;     // main brightness level, 0-255, controls all RGB LEDs
  uint8_t redBrightnessScale   = 255;     // sets brightness scaling, 0-255, for balancing R G B elements
  uint8_t greenBrightnessScale = 255;
  uint8_t blueBrightnessScale  = 255;

  elapsedMillis updateTimer, agioHelloTimeoutTimer, gpsUpdateTimeoutTimer;

  uint16_t updatePeriod;
  bool blinkNextLoop;
  
  bool blueFlashState, blueFlashEnabled;
  uint16_t blueFlashPeriod = 100;
  uint32_t blueFlashStartTime;
  uint32_t blueFlashStopTime;

  const char* ledNames[4] = {
    "PWR_ETH",
    "GPS",
    "STEER",
    "UNUSED"
  };



public:

  struct LED_DATA {
    uint8_t stage;           // stores hierarchy of stages, bit0 - red solid, bit1 - red blink, bit2 - green blink, bit3 - green solid, highest ON bit determines color
    uint8_t redValue;        // stores red value while LED is OFF during a blink stage
    uint8_t greenValue;
    uint8_t blueValue;
    //bool blinkState;
    bool blueFlash;
  }; LED_DATA data[4];

  LEDS(uint16_t _updatePeriod) {
    updatePeriod = _updatePeriod;
    init();
  }
  
  LEDS(uint16_t _updatePeriod, uint8_t _redScale, uint8_t _greenScale, uint8_t _blueScale) {
    updatePeriod = _updatePeriod;
    redBrightnessScale = _redScale;
    greenBrightnessScale = _greenScale;
    blueBrightnessScale = _blueScale;
    init();
  }
  ~LEDS(void) {}

  void init() {
    #ifdef AIOv4x
      for (uint8_t i = 0; i < NUM_LEDS; i++){
        pinMode(v4redLEDio[i], OUTPUT);
        pinMode(v4grnLEDio[i], OUTPUT);
      }
    #endif
    pinMode(GGA_LED, OUTPUT);
    WS2811.begin();
    delay(1);
    WS2811.clear();
    delay(1);
    updateLoop();
  }

  void toggleTeensyLED() {
    digitalWrite(GGA_LED, !digitalRead(GGA_LED));
  }

  void setGpsLED(uint8_t _fixState, bool _debug = false)
  {
    gpsUpdateTimeoutTimer = 0;

    switch (_fixState) {
      case 0:                         // 0: Fix not valid
        set(LED_ID::GPS, STAGE1_RED, _debug);
        return;
      case 1:                         // 1: GPS fix
        set(LED_ID::GPS, STAGE2_RED_BLINK, _debug);
        return;
      case 2:                         // 2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
        set(LED_ID::GPS, STAGE2_RED_BLINK, _debug);
        return;
      case 4:                         // 4: RTK Fixed, xFill
        set(LED_ID::GPS, STAGE4_GREEN, _debug);
        return;
      case 5: case 6:                 // 5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX - 6: INS Dead reckoning
        set(LED_ID::GPS, STAGE3_GREEN_BLINK, _debug);
        return;
      case 8:                         // 8: Simulation mode
        set(LED_ID::GPS, STAGE4_GREEN, _debug);
        return;
      case 3: case 7: default:        // 3: Not applicable, 7: Manual input mode
        set(LED_ID::GPS, STAGE0_OFF, true);
        return;
    }
  }

  void set(uint8_t _id, uint8_t _stage, bool _debug = false)
  {
    if (_id == LED_ID::PWR_ETH && _stage == AGIO_CONNECTED) agioHelloTimeoutTimer = 0;
    if (_stage == data[_id].stage) return;
    
    if (_stage == STAGE3_GREEN_BLINK || _stage == STAGE4_GREEN ) {
      data[_id].redValue = 0;
      data[_id].greenValue = greenBrightnessScale;
    } else if (_stage == STAGE1_RED || _stage == STAGE2_RED_BLINK) {
      data[_id].redValue = redBrightnessScale;
      data[_id].greenValue = 0;
    } else {                            // any other value, set OFF (stage: 0)
      _stage = 0;
      data[_id].redValue = 0;
      data[_id].greenValue = 0;
    }

    // not needed?, is set in updateLoop
    WS2811.setPixelColor(_id, data[_id].redValue, data[_id].greenValue, 0);
    // not needed?, is set in updateLoop

    //if (_debug && data[_id].stage != _stage) Serial.printf("\nSetting LED:%1i from stage %1i to %1i", _id, data[_id].stage, _stage);
    if (_debug) Serial.printf("\nChanging %s LED from stage %1i to %1i", ledNames[_id], data[_id].stage, _stage);
    data[_id].stage = _stage;
  }

  void updateV4LEDS()
  {
    #ifdef AIOv4x
    for (byte i = 0; i < NUM_LEDS; i++) {
      if ((data[i].stage == STAGE2_RED_BLINK || data[i].stage == STAGE3_GREEN_BLINK) && blinkNextLoop) {
        digitalWrite(v4redLEDio[i], 0);
        digitalWrite(v4grnLEDio[i], 0);
      } else {
        digitalWrite(v4redLEDio[i], (data[i].redValue > 0 ? 1 : 0));
        digitalWrite(v4grnLEDio[i], (data[i].greenValue > 0 ? 1 : 0));
      }
    }
    #endif
  }

  void updateLoop()
  {
    if (agioHelloTimeoutTimer > 5000) set(LED_ID::PWR_ETH, ETH_READY, true); // sets PWR__ETH LED to green_blink if AgIO Hello times out after 5s
    if (gpsUpdateTimeoutTimer > 3000) setGpsLED(3, true);                    // sets GPS LED OFF if no GPS update for 3s

    if (updateTimer > updatePeriod) {
      updateTimer = 0;

      // update rgb values from each RGB_DATA struct, brightness scaling values & send down the wire
      WS2811.setBrightness(mainBrightness);

      for (byte i = 0; i < NUM_LEDS; i++) {
        if ((data[i].stage == STAGE2_RED_BLINK || data[i].stage == STAGE3_GREEN_BLINK) && blinkNextLoop) {
          WS2811.setPixelColor(i, 0, 0, 0);
        } else {
          WS2811.setPixelColor(i, data[i].redValue, data[i].greenValue, 0);
        }
      }
      blinkNextLoop = !blinkNextLoop;
      if (!blinkNextLoop) {     // remove "if" to additionally flash blue during ON stage of blink cycle
        blueFlashStartTime = millis() + updatePeriod / 2 - blueFlashPeriod / 2;
        blueFlashState = 0;
        blueFlashEnabled = 1;
      }

      WS2811.show();
      updateV4LEDS();
    }

    if (blueFlashEnabled)
    {
      if (!blueFlashState && millis() > blueFlashStartTime)
      {
        blueFlashState = 1;

        for (byte i = 0; i < NUM_LEDS; i++)
        {
          if (data[i].blueFlash) WS2811.setPixelColor(i, 0, 0, blueBrightnessScale);
        }
        WS2811.show();
      }

      if (blueFlashState && millis() > blueFlashStartTime + blueFlashPeriod)
      {
        //blueFlashState = 0;
        blueFlashEnabled = 0;

        for (byte i = 0; i < NUM_LEDS; i++)
        {
          if (data[i].blueFlash) {
            // add "&& !blinkNextLoop" to additionally flash blue during ON stage of blink cycle
            if ((data[i].stage == STAGE2_RED_BLINK || data[i].stage == STAGE3_GREEN_BLINK)) {// && !blinkNextLoop) {
              WS2811.setPixelColor(i, 0, 0, 0);
            } else {
              WS2811.setPixelColor(i, data[i].redValue, data[i].greenValue, 0);
            }
            data[i].blueFlash = 0;
          }
        }
        WS2811.show();
      }
    }
  }

  void setBrightness(uint8_t _brightness){
    mainBrightness = _brightness;
  }

  // queue LED to flash blue at next timed cycle
  void queueBlueFlash(uint8_t _id) {
    data[_id].blueFlash = 1;
  }

  // instantly flash LED blue
  void activateBlueFlash(uint8_t _id) {
    // add code to instantly flash blue instead of waiting for next timed cycle
    data[_id].blueFlash = 1;
  }

/*  void printBinary(int var) {
    for (unsigned int mask = B1000; mask; mask >>= 1) {    // prints 4 bits/digits, set mask to the number of bits you want to print
      Serial.write(var & mask ? '1' : '0');
    }
  }*/


};
#endif