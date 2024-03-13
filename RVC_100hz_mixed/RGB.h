#include <stdint.h>
//#include <stdint.h>
//#include "wiring.h"
//#include "Arduino.h"
/*
  This is a library written for AgOpenGPS AIO PCBs with RGB faceplate LEDs
  - written by Matt Elias, 2024

  It uses the Adafruit_NeoPixel library to control (4) faceplate facing RGB LEDs via (4) WS2811 chips using NRZ protocol

  The goal of this class is to retrieve states from other classes (rtkStatus, ethStatus, steerStatus, etc)
  and decode those values into RGB color/blink states. That way all the RGB/LED code stays in this class
  instead of strewn about all over the rest of the code. I hope this works! ;)

*/

#ifndef H_RGB_H
#define H_RGB_H

#include "elapsedMillis.h"
#include <Adafruit_NeoPixel.h>  // https://github.com/adafruit/Adafruit_NeoPixel/blob/master/Adafruit_NeoPixel.h

class RGB {
private:
  const uint8_t WS2811_PIN = 33;
  const uint8_t NUM_LEDS = 4;
  Adafruit_NeoPixel WS2811 = Adafruit_NeoPixel(NUM_LEDS, WS2811_PIN, NEO_BGR + NEO_KHZ800);

  uint8_t mainBrightness       = 255;     // main brightness level, 0-255, controls all RGB LEDs
  uint8_t redBrightnessScale   = 255;     // sets brightness scaling, 0-255, for balancing R G B elements
  uint8_t greenBrightnessScale = 255;
  uint8_t blueBrightnessScale  = 255;

  elapsedMillis updateTimer;
  uint16_t updatePeriod;
  bool blinkNextLoop;
  
  bool blueFlashState, blueFlashEnabled;
  uint16_t blueFlashPeriod = 50;
  uint32_t blueFlashStartTime;
  uint32_t blueFlashStopTime;

public:
  typedef enum {
    PWR_ETH,
    GPS,
    STEER,
    UNUSED
  } LED_ID;
  
  enum {
    STAGE0_OFF,
    STAGE1_RED,
    STAGE2_RED_BLINK,
    STAGE3_GREEN_BLINK,
    STAGE4_GREEN,
  };

  struct RGB_DATA {
    uint8_t stage;           // stores hierarchy of stages, bit0 - red solid, bit1 - red blink, bit2 - green blink, bit3 - green solid, highest ON bit determines color
    uint8_t redValue;        // stores red value while LED is OFF during a blink stage
    uint8_t greenValue;
    uint8_t blueValue;
    //bool blinkState;
    bool blueFlash;
  }; RGB_DATA data[4];

  RGB(uint16_t _updatePeriod) {
    updatePeriod = _updatePeriod;
    init();
  }
  
  RGB(uint16_t _updatePeriod, uint8_t _redScale, uint8_t _greenScale, uint8_t _blueScale) {
    updatePeriod = _updatePeriod;
    redBrightnessScale = _redScale;
    greenBrightnessScale = _greenScale;
    blueBrightnessScale = _blueScale;
    init();
  }
  ~RGB(void) {}

  void init() {
    WS2811.begin();
    delay(1);
    WS2811.clear();
    delay(1);
    updateLoop();
  }

  void setGpsLED(uint8_t _fixState, bool _debug = false) {
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

  void set(uint8_t _id, uint8_t _stage, bool _debug = false) {
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
    WS2811.setPixelColor(_id, data[_id].redValue, data[_id].greenValue, 0);
    if (_debug && data[_id].stage != _stage) Serial.printf("\nSetting LED:%1i from stage %1i to %1i", _id, data[_id].stage, _stage);
    data[_id].stage = _stage;
  }

  void updateLoop() {
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
    }

    if (blueFlashEnabled)
    {
      if (!blueFlashState && millis() > blueFlashStartTime)
      {
        blueFlashState = 1;

        for (byte i = 0; i < NUM_LEDS; i++) {
          if (data[i].blueFlash) {
            WS2811.setPixelColor(i, 0, 0, blueBrightnessScale);
          }
        }
        WS2811.show();
      }

      if (blueFlashState && millis() > blueFlashStartTime + blueFlashPeriod)
      {
        //blueFlashState = 0;
        blueFlashEnabled = 0;

        for (byte i = 0; i < NUM_LEDS; i++) {
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
    // add function to instantly flash blue?
    data[_id].blueFlash = 1;
  }

/*  void printBinary(int var) {
    for (unsigned int mask = B1000; mask; mask >>= 1) {    // prints 4 bits/digits, set mask to the number of bits you want to print
      Serial.write(var & mask ? '1' : '0');
    }
  }*/


};
#endif