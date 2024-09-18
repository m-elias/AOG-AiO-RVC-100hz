//
// connection plan:
// Teensy Serial 5 RX (21) to F9P Position receiver TX1 (Position data)
// Teensy Serial 5 TX (20) to F9P Position receiver RX1 (RTCM data for RTK)
// Teensy Serial 8 RX (34) to F9P Heading receiver TX1 (Relative position from left antenna to right antenna)
// Teensy Serial 8 TX (35) to F9P Heading receiver RX1
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

#define AIOv50a
const char inoVersion[] = "AiO v5.0a OGX - " __DATE__;

// ********* IO Defines *********
const uint8_t WAS_SENSOR_PIN = A15;     // WAS input
const uint8_t SPEEDPULSE_PIN = 18;      
const uint8_t SPEEDPULSE10_PIN = 19;   // 1/10 speedpulse output, strictly for human visualization
#include "misc.h"
SpeedPulse speedPulse(SPEEDPULSE_PIN, SPEEDPULSE10_PIN);     // misc.h

const uint8_t PIEZO1 = 37;
const uint8_t PIEZO2 = 36;

// Cytron/DRV8701
#define DIR_PIN           6     // DRV Dir pin
#define PWM_PIN           9     // DRV PWM pin
#define SLEEP_PIN         4     // DRV Sleep pin, LOCK output

// Switches/Sensors
#define STEER_PIN         2
#define WORK_PIN        A17
#define KICKOUT_D_PIN     3     // REMOTE
#define CURRENT_PIN     A13     // CURRENT sense from on board DRV8701
#define KICKOUT_A_PIN   A12     // PRESSURE

// ********* Serial Assignments *********
#define SerialRTK Serial3               // RTK radio
//HardwareSerial SerialRTK = Serial3;   // causes boot loop

HardwareSerial* SerialIMU = &Serial6;   // IMU BNO-085 in RVC serial mode

// HardwareSerial *SerialGPS = &Serial5;   // Main postion receiver (GGA & VTG)
#define SerialGPS1 Serial5

// HardwareSerial *SerialGPS2 = &Serial8; // Dual heading receiver  (relposNED)
#define SerialGPS2 Serial8

// HardwareSerial *SerialRS232 = &Serial7; // RS232
#define SerialRS232 Serial7

// HardwareSerial *SerialESP32 = &Serial2; // ESP32
#define SerialESP32 Serial2

//const int32_t baudGPS = 921600;
const int32_t baudGPS = 460800;
const int32_t baudRTK = 460800;     // most are using Xbee radios with default of 115200
const int32_t baudRS232 = 38400;
const int32_t baudESP32 = 460800;

#else
There has been a PCB/hardware selection error, only select one HWxxx.h file
#endif