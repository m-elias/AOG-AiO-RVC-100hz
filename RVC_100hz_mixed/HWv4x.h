//
// connection plan:
// Teensy Serial 7 RX (28) to F9P Position receiver TX1 (Position data)
// Teensy Serial 7 TX (29) to F9P Position receiver RX1 (RTCM data for RTK)
// Teensy Serial 2 RX (07) to F9P Heading receiver TX1 (Relative position from left antenna to right antenna)
// Teensy Serial 2 TX (08) to F9P Heading receiver RX1
// F9P Position receiver TX2 to F9P Heading receiver RX2 (RTCM data for Moving Base)
//
// Configuration of receiver
// Position F9P
//   CFG-RATE-MEAS - 100 ms -> 10 Hz
//   CFG-UART1-BAUDRATE 460800
//     UART1 In - RTCM (Correction Data from AOG)
//     UART1 Out - NMEA GGA
//   CFG-UART2-BAUDRATE 460800
//     UART2 Out - RTCM 1074,1084,1094,1230,4072.0 (Correction data for Heading F9P, Moving Base)  
//                 1124 is not needed (China's BeiDou system) - Save F9P brain power 
//
// Heading F9P
//   CFG-RATE-MEAS - 100 ms -> 10 Hz
//   CFG-UART1-BAUDRATE 460800
//     UART1 Out - UBX-NAV-RELPOSNED
//   CFG-UART2-BAUDRATE 460800
//     UART2 In RTCM

#ifndef Hardware_H
#define Hardware_H

#define AIOv4x
const char inoVersion[] = "AiO v4.x";

// ********* IO Defines *********
const uint8_t WAS_SENSOR_PIN = A0;      // WAS input
const uint8_t SPEEDPULSE_PIN = 36;
#include "misc.h"
SpeedPulse speedPulse(SPEEDPULSE_PIN);     // misc.h

// Cytron
#define DIR_PIN           4     // IBT2 Enable pin
#define PWM_PIN           2     // IBT2 Left pin
#define SLEEP_PIN         3     // IBT2 Right pin

// Switches/Sensors
#define STEER_PIN        32
#define WORK_PIN         34
#define KICKOUT_D_PIN    37     // REMOTE input
#define CURRENT_PIN     A17     // ACS CURRENT input from Cytron module
#define KICKOUT_A_PIN   A10     // PRESSURE input

#define I2C_WIRE Wire1          // used for ADS1115 wheel angle sensor

// ********* Serial Assignments *********
HardwareSerial* SerialIMU = &Serial5;   // IMU BNO-085 in RVC serial mode
#define SerialRTK  Serial3               // RTK radio
#define SerialGPS1 Serial7              // Main postion receiver (GGA & VTG)
#define SerialGPS2 Serial2              // Dual heading receiver  (relposNED)

const int32_t baudGPS = 460800;
const int32_t baudRTK = 115200;     // most are using Xbee radios with default of 115200

#else
There has been a PCB/hardware selection error, only select one HWxxx.h file
#endif