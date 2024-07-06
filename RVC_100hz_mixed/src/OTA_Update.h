#ifndef OTA_Update_h
#define OTA_Update_h
#include "Arduino.h"
#include <AsyncWebServer_Teensy41.h>
#include "FXUtil.h"		// read_ascii_line(), hex file support
extern "C" {
  #include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives
}

class OTA_Update {
public:
    OTA_Update();
    void OTAapply();

private:
    
};





#endif