/*
  This is a library written for the Teensy 4.1 microcontroller

  Written by Matt Elias, 2023

  It's used to reboot (short press) or reset (long press)
  Pressing the button shorter then longPressPeriod causes the LED to blink and reboots the Teensy
  Holding the button longer then longPressPeriod causes the LED to flash and upate() returns true
*/

#ifndef RESET_H
#define RESET_H

class RESET {
  public:
    RESET(uint8_t _btnIO, uint16_t _longPressPeriod = 10000, uint8_t _ledIO = LED_BUILTIN){
      btnIO = _btnIO;
      pinMode(btnIO, INPUT_PULLUP);
      longPressPeriod = _longPressPeriod;
      ledIO = _ledIO;
      pinMode(ledIO, OUTPUT);
    }
    ~RESET(void) {}                      //destructor

    bool update() {
      if (digitalRead(btnIO) == LOW){
        uint32_t btnPressTime = millis();
        digitalWrite(ledIO, HIGH);
        while (digitalRead(btnIO) == LOW && millis() - btnPressTime <= longPressPeriod){
          digitalWrite(ledIO, !digitalRead(ledIO));
          delay(70);
        }
        if (millis() - btnPressTime >= longPressPeriod){  // btn held for more than longPressPeriod (10s default)
          while (digitalRead(btnIO) == LOW){  // flash LED while btn is held to indicate factory/firmware reset is triggered
            if (digitalRead(ledIO) == 1) delay(40);
            else delay(10);
            digitalWrite(ledIO, !digitalRead(ledIO));
          }
          digitalWrite(ledIO, LOW);
          return true;
        } else {  // short btn press, do reboot instead of settings reset
          reboot();
        }
      }
      return false;
    }

    void reboot(bool _pause = false){
      if (_pause) delay(100);   // delay for serial/etc to finish
      SCB_AIRCR = 0x05FA0004;   // Teensy Reboot
    }

  private:
    uint8_t btnIO;
    uint16_t longPressPeriod;
    uint8_t ledIO;
};

#endif