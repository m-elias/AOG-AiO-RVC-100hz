#include <stdint.h>
#include "usb_serial.h"
/*!
 * 	This is a library for use with thethe Adafruit BNO08x breakout:
 * 	https://www.adafruit.com/products/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 */

#ifndef _BNO_rvc_H
#define _BNO_rvc_H

constexpr auto MILLI_G_TO_MS2 = 0.0098067; ///< Scalar to convert milli-gs to m/s^2;
constexpr auto DEGREE_SCALE = 0.1;        ///< To convert the degree values

#include "Arduino.h"
#include <Wire.h>

typedef struct BNO_rvcData {
    int16_t yawX10,     ///< Yaw in Degrees x 10
        pitchX10,     ///< Pitch in Degrees x 10
        rollX10,       ///< Roll in Degrees x 10
        yawX100,         // yaw in original x100
        angVel;         //running total of angular velocity
} BNO_rvcData;


class BNO_rvc {

private:
  HardwareSerial *serial_dev;
  int16_t prevYAw;

public:
  uint32_t angCounter;
  bool isSwapXY = false;

  bool begin(HardwareSerial* theSerial)
  {
    serial_dev = theSerial;
    serial_dev->begin(115200);  // This is the baud rate specified by the BNO datasheet for serial RVC
    elapsedMillis timeout = 0;
    BNO_rvcData testData;
    while (!read(&testData))   // wait/check for BNO RVC serial data
    {
      if (timeout > 25)   // data should arrive every 10ms
      {
        Serial.print("\r\n-* No BNO-085 in RVC mode detected *");
        return false;
      }
    }
    
    Serial.print("\r\n- BNO-085 RVC detected");
    Serial.printf("\r\n  - yaw:%i , pitch:%i , roll:%i", testData.yawX10, testData.pitchX10, testData.rollX10);
    return true;
  }

  bool read(BNO_rvcData* bnoData)
  {
    //read the 16 byte sentence AA AA Index Yaw Pitch Roll LSB MSB

    if (!serial_dev->available()) return false;

    if (serial_dev->peek() != 0xAA)
    {
      serial_dev->read();
      return false;
    }

    // Now read all 19 bytes
    if (serial_dev->available() < 19) return false;

    // at this point we know there's at least 19 bytes available and the first is AA
    if (serial_dev->read() != 0xAA) return false;

    // make sure the next byte is the second 0xAA
    if (serial_dev->read() != 0xAA) return false;

    uint8_t buffer[19];
    if (!serial_dev->readBytes(buffer, 17)) return false;

    // get checksum ready
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 16; i++) sum += buffer[i];

    if (sum != buffer[16]) return false;

    //clean out any remaining bytes in case teensy was busy
    uint16_t extra = serial_dev->available();
    if (extra > 0) { Serial.print((String)"\r\n" + millis() + " *** BNO serial input buffer had " + extra + " bytes leftover! ***"); }
    while (serial_dev->available() > 0) serial_dev->read();

    //Serial.print((String)"\r\nBNO update " + millis());

    int16_t temp;

    if (angCounter < 20)
    {
      temp = buffer[1] + (buffer[2] << 8);
      bnoData->yawX100 = temp; //For angular velocity calc
      bnoData->angVel += (temp - prevYAw);
      angCounter++;
      prevYAw = temp;
    }
    else
    {
      angCounter = 0;
      prevYAw = bnoData->angVel = 0;
      temp = 0;
    }

    bnoData->yawX10 = (int16_t)((float)temp * DEGREE_SCALE);
    if (bnoData->yawX10 < 0) bnoData->yawX10 += 3600;

    temp = buffer[3] + (buffer[4] << 8);
    bnoData->pitchX10 = (int16_t)((float)temp * DEGREE_SCALE);

    temp = buffer[5] + (buffer[6] << 8);
    bnoData->rollX10 = (int16_t)((float)temp * DEGREE_SCALE); //Confirmed X as stock direction of travel

    return true;
  }

};

#endif