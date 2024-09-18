//#include "NativeEthernet.h"

/* An attempt to migrate OpenGradeX's module code into a single class/library for use with AIO modules
    by Matt Elias Aug 2024

  This class only takes care of the OGX leveling logic & PGN parsing
    - uses a callback function for the hardware interaction (ie Arduino digital/analog write or MCP4725 etc)
      - the host microcontroller has to take care of the UDP & hardware connections

  to do:
    - use callback to sent data to OGX
    - add IMU support
*/

#ifndef OGX_H
#define OGX_H

#include <Arduino.h>
#include <EEPROM.h>
#include "elapsedMillis.h"
//#include "IPAddress.h"

class OpenGradeX
{
private:

  typedef void (*ExternalHandler)(void);
  ExternalHandler Output1_Handler = NULL;

  using ReplyHandler = void (*)(const uint8_t*, uint8_t, IPAddress);
  ReplyHandler UDPReplyHandler = NULL;

  // version numbers to report to OGX which expects there to be two ESP32 modules instead of one AIO module
  const char *versionAnt = "2.1.3.A";
  const char *versionGrade = "1.4.2.G";

  //EEPROM
  #define EE_ID 17   // Change this number to reset and reload default parameters To EEPROM
  int16_t eeAddr = -1;  // -1 defaults to no EEPROM saving/loading
  bool isInit;

  // changed IPs to match AOG enviro
  // using one AIO module instead of two ESP32 for OGX so they share the same module IP
  const IPAddress openGradeIP = { 192, 168, 5, 9 };       //OpenGradeX Server
  const IPAddress gradeControlIP = { 192, 168, 5, 126 };  // GradeControl Module IP
  const IPAddress antennaIP = { 192, 168, 5, 126 };       // Antenna Module IP

  //UDP HEADERS
  #define DATA_HEADER 10001
  #define SETTINGS_HEADER 10002
  #define GPS_HEADER 10003
  #define IMU_HEADER 10004
  #define NTRIP_HEADER 10005
  #define RESET_HEADER 10100
  #define SYSTEM_HEADER 10101
  #define WIFI_HEADER 10102

  // Valve Definitions
  #define VALVE_NEUTRAL 2048
  #define CNH 0
  #define DEERE 1
  #define DANFOSS 2
  const char* valveNames[3] = { "CNH", "JD", "Danfoss" };
  const double RET_MIN[3] = { 0.11, 0.11, 0.26 }; // CNH, JD, Danfoss
  const double EXT_MAX[3] = { 0.89, 0.89, 0.74 };

  struct Valve {
    uint8_t Kp = 38;          // PID params, using Black Ace's values as defaults, overwritten from OGX app and saved in EEPROM
    uint8_t Ki = 20; 
    uint8_t Kd = 28; 
    uint8_t retDeadband = 10;  // blade retract deadband, 10 is 0.25V (default) on CNH/JD SCV control
    uint8_t extDeadband = 10;  // blade extend deadband
    uint8_t type = 0;         // 0= CNH    1= Deere     2= Danfoss

    bool operator==(const Valve &other) const {
      return this->Kp == other.Kp
        && this->Ki == other.Ki
        && this->Kd == other.Kd
        && this->retDeadband == other.retDeadband
        && this->extDeadband == other.extDeadband
        && this->type == other.type;
    }

    bool operator!=(const Valve &other) const {
      return !(*this == other);
    }
  };  Valve config;      //6 bytes

  /////////// PID SETTINGS ////////////
  // set these in OGX app
  float Kp;          // Black Ace's was 38
  float Ki;          // Black Ace's was 20 (0.02)
  float Kd;          // Black Ace's was 28 (2800)
  float delta_setpoint = 0;  

  bool b_autoState = false;
  uint8_t b_deltaDir = 0;
  double b_cutDelta = 0.0;
  byte b_bladeOffsetOut = 0, b_deadband = 0;

  /////////////// CNH Valve /////////////////////////
  uint16_t analogOutput1 = VALVE_NEUTRAL; //send to MCP4725
  uint16_t analogOutput2 = VALVE_NEUTRAL; //send to MCP4725, not used, always valve neutral
  double voltage = 0; // diagnostic Voltage
  double voltage2 = 0;
  int retDeadband = 1845;
  int extDeadband = 2250;
  int retMin = (0.11 * 4096);   //450.56  CNH 
  int extMax = (0.89 * 4096);   //3645
  bool isAutoActive = false;
  bool isCutting = false;

  //loop time variables in milliseconds
  const uint16_t LOOP_TIME = 50;          // 20hz control loop
  elapsedMillis loopTimer;
  int8_t ledPin = -1;
  bool ledPol = 1;
  
  //Communication with OpenGradeX
  bool isOGXConnected = false;
  bool isDataFound = false, isSettingFound = false;
  unsigned long watchdogTimer = 0;   //make sure we are talking to OGX
  const uint32_t OGXTimeout = 20;   // 1 sec timeout at 20hz loop

public:
  uint8_t debugLevel = 5;
    // 0 - debug prints OFF
    // 1 - alerts/errors only
    // 2 - init info
    // 3 - config PGN announcements (only sent when changing settings in AOG)
    // 4 - all PGN announcements
    // 5 - all PGN data

  // OGX sends from 9998
  #define OGX_PORT 9997         // OpenGradeX Server listening Port, originally 9999 but that interferes with AOG so changed to 9997
  #define GRADE_PORT 7777       // GradeControl listening Port
  //#define POSITION_PORT 7777  // Antenna listening Port, originally 8888 but interferes with AOG so changed to 7777 (combined with grade port/module)

  void init(int16_t _eeAddr = -1, int8_t _ledPin = -1, bool _ledPol = 1, const uint8_t _eeSize = 10) // 10 bytes of EEPROM reserved, only 6 bytes used
  {
    ledPin = _ledPin;
    ledPol = _ledPol;
    pinMode(ledPin, OUTPUT);

    eeAddr = _eeAddr;
    loadFromEeprom();

    // trigger output callback to set all outputs to OFF

    isInit = true;
  }

  void updateLoop()
  {
    if (loopTimer >= LOOP_TIME) // 20 HZ
    {
      loopTimer = 0;
      watchdogTimer++;

      if (watchdogTimer > OGXTimeout){
        isOGXConnected = false;
        if (ledPin >= 0) digitalWrite(ledPin, !ledPol);
      }    
      else
      {
        isOGXConnected = true;
        if (ledPin >= 0) digitalWrite(ledPin, ledPol);
      }

      (watchdogTimer > OGXTimeout*5000)? watchdogTimer = 150 : watchdogTimer; // Prevent overflow

      calcPID();
      SendDataToOGX();
    }
  }

  // collect one nmea char/byte at a time until '\n' is detected, then send to OGX
  void nmeaInput(uint8_t _gpsByte)
  {
    static uint8_t index = 0;
    static char nmeaBuffer[150];
    nmeaBuffer[index++] = _gpsByte; // build the nmea sentence
    //Serial.write(_gpsByte);

    if (_gpsByte == '\n')
    {
      nmeaBuffer[index + 1] = 0;
      //Serial << "\r\nLF detected, sending buffer to OGX: " << index;
      //Serial << "\r\n-"; Serial.write(nmeaBuffer);

      UDP.PGN_OGX.beginPacket(openGradeIP, OGX_PORT);   //Initiate transmission of data
      UDP.PGN_OGX.print(GPS_HEADER);
      UDP.PGN_OGX.print(",");
      UDP.PGN_OGX.print(nmeaBuffer);
      UDP.PGN_OGX.endPacket();  // Close communication

      index = sizeof(nmeaBuffer); // trigger buffer reset below
    }
    
    if (index >= sizeof(nmeaBuffer) - 2)  // protect against buffer overruns & reset after sending to OGX
    {
      index = 0;
      memset(nmeaBuffer, 0, sizeof(nmeaBuffer));
    }
  }

  void checkforPGNs(char* udpBuffer, uint16_t len)
  {
      bool pgnMatched = false;

      Serial << "\r\n" << millis() << " OGX packet ";
      //Serial.print(UDP.PGN_OGX.remoteIP()); Serial << ":";
      //Serial.print(UDP.PGN_OGX.remotePort()); Serial << " ";
      Serial << "(";
      if (len < 10) Serial << "0";
      Serial << len << ") ";//" << UDP.PGN_OGX.available() << ") ";

      char *ptr = strtok(udpBuffer, ",");  // takes a list of delimiters
      byte numParams = 0;
      char *strings[1460] = { };

      while(ptr != NULL){
        strings[numParams] = ptr;
        //Serial.print(ptr); Serial.print(" ");
        //Serial.print(strings[numParams]); Serial.print(" ");
        numParams++;
        ptr = strtok(NULL, ",");  // split comma delimited char arrays
      }

      if (numParams == 0) return;

      uint16_t header = atoi(strings[0]);
      Serial << "PGN " << header << " (" << numParams << ") ";

      for(int n = 0; n < numParams; n++){ 
        Serial.print(strings[n]); Serial.print(" ");
      }

      if (header == DATA_HEADER) {  // 10001 DATA
        //printPgnAnnoucement(header, (char*)"DATA", len, numParams);
        watchdogTimer = 0;
        b_deltaDir =  atoi(strings[1]);       // Cut Dir
        b_autoState = atoi(strings[2]);       // Auto State
        b_cutDelta =  atoi(strings[3]) * 0.1; // Cut Delta

        /*Serial << "\r\n- Blade Dir   " << b_deltaDir;
        Serial << "\r\n- Auto State  " << b_autoState;
        Serial << "\r\n- Blade Delta " << b_cutDelta;*/

        if (b_deltaDir == 3) isCutting = false;
        else isCutting = true;

        if (b_autoState == 1) isAutoActive = true;
        else isAutoActive = false;

        pgnMatched = true;
      }

      if (header == SETTINGS_HEADER) {  // 10002 SETTINGS
        printPgnAnnoucement(header, (char*)"SETTINGS", len, numParams);

        Valve oldConfig = config;

        config.Kp = atoi(strings[1]);
        config.Ki = atoi(strings[2]);
        config.Kd = atoi(strings[3]);
        config.retDeadband = atoi(strings[4]);
        config.extDeadband = atoi(strings[5]);
        config.type = atoi(strings[6]);

        setWorkingVars();

        printConfig();

        if (config != oldConfig) saveToEeprom();
        pgnMatched = true;
      }
      
      if (header == GPS_HEADER) {  // 10003 GPS, should not receive this, sent from Ant module only
        printPgnAnnoucement(header, (char*)"GPS", len, numParams);
        pgnMatched = true;
      }
      
      if (header == IMU_HEADER) {  // 10004 IMU
        printPgnAnnoucement(header, (char*)"IMU zero", len, numParams);
        pgnMatched = true;
      }
      
      if (header == NTRIP_HEADER) {  // 10005 NTRIP
        printPgnAnnoucement(header, (char*)"NTRIP", len, numParams);
        pgnMatched = true;
      }
      
      if (header == RESET_HEADER) {  // 10100 RESET
        printPgnAnnoucement(header, (char*)"RESET", len, numParams);
        pgnMatched = true;
      }
      
      if (header == SYSTEM_HEADER) {   // 10101 PING/SYSTEM
        //printPgnAnnoucement(header, (char*)"PING", len, numParams);
        if (atoi(strings[1]) != 0) { // module ping
          static bool moduleReplyToggle = 0;

          if (moduleReplyToggle) {    // OGX sends ping to two ESP32 modules but I've combined it into one so we alternate replies
            // send antenna module reply
            //Serial << "\r\nReply to OGX: from antenna module\r\n";
            UDP.PGN_OGX.beginPacket(openGradeIP, OGX_PORT);   //Initiate transmission of data
            UDP.PGN_OGX.print(SYSTEM_HEADER);
            UDP.PGN_OGX.print(",");
            UDP.PGN_OGX.print(155);   // antenna module ID
            UDP.PGN_OGX.print(",");
            UDP.PGN_OGX.print(versionAnt);     
            UDP.PGN_OGX.endPacket();  // Close communication
          } else {
            // send grade control module reply
            //Serial << "\r\nReply to OGX: from grade control module\r\n";
            UDP.PGN_OGX.beginPacket(openGradeIP, OGX_PORT);   //Initiate transmission of data
            UDP.PGN_OGX.print(SYSTEM_HEADER);
            UDP.PGN_OGX.print(",");
            UDP.PGN_OGX.print(255);   // grade control module ID
            UDP.PGN_OGX.print(",");
            UDP.PGN_OGX.print(versionGrade);     
            UDP.PGN_OGX.endPacket();  // Close communication    
          }
          moduleReplyToggle = !moduleReplyToggle;

          pgnMatched = true;
        }
      }
      
      if (header == WIFI_HEADER) {  // 10102 WIFI
        printPgnAnnoucement(header, (char*)"WIFI", len, numParams);
        if (atoi(strings[1]) == 1) { // request Wifi scan
          UDP.PGN_OGX.beginPacket(openGradeIP, OGX_PORT);   //Initiate transmission of data
          UDP.PGN_OGX.print(WIFI_HEADER);
          UDP.PGN_OGX.print(",");
          UDP.PGN_OGX.print("No WIFI: using Ethernet");
          UDP.PGN_OGX.print(",");
          UDP.PGN_OGX.print("AiO Teensy");
          UDP.PGN_OGX.endPacket();  // Close communication    
        } else if (atoi(strings[1]) == 2) { // set Wifi creds
          Serial << "\r\n- Wifi creds: " << strings[2] << ":" << strings[3];
        }

        pgnMatched = true;
      }

      if (!pgnMatched) printPgnAnnoucement(header, (char*)"*UNKNOWN*", len, numParams);
  }


  void setOutput1Handler(ExternalHandler _extHandler) {
    Output1_Handler = _extHandler;
  }

  uint16_t getAnalog1()
  {
    return analogOutput1;
  }

  void setUdpReplyHandler(ReplyHandler _replyHandler) {
    UDPReplyHandler = _replyHandler;
  }

private:

  void calcPID()
  {
    static double PID_i = 0, delta_previous_error = 0;

    if (isAutoActive && isCutting && isOGXConnected) {
      analogOutput1 = VALVE_NEUTRAL;  
      static float cut1;  // i think cut1 can be eliminated by setting b_cutDelta as +- in PGN parser and use b_cutDelta here instead

      if (b_deltaDir == 0) cut1 = -b_cutDelta;
      else cut1 = b_cutDelta;

      double delta_error = delta_setpoint - cut1;

      if(-b_deadband < delta_error && delta_error < b_deadband) {  // if deadband is set for 0, this never passes
        PID_i = PID_i + (Ki * delta_error); // calculate the i error
        Serial << "\r\n probably never see this because b_deadband is always 0";
      } else
        PID_i = 0;

      double PID_total = (Kp * delta_error)                                       // P
                       + PID_i                                                    // I
                       + (Kd*((delta_error - delta_previous_error) / LOOP_TIME)); // D

      if (PID_total >  2300) PID_total = 2300;      
      if (PID_total <  -2300) PID_total = -2300;

      if (b_deltaDir == 1){ // Delta is Positive need to lower blade (RETRACT)
        analogOutput1 = map(PID_total, 0.0, -2300, retDeadband , retMin);
      }
      else if (b_deltaDir == 0){ // Delta is Negative need to raise blade (EXTEND)
        analogOutput1 = map(PID_total,  0.0, 2300, extDeadband, extMax);
      }
      
      if (analogOutput1 >= extMax) analogOutput1 = extMax; // do not exceed 4096
      if (analogOutput1 <= retMin) analogOutput1 = retMin; // do not write negative numbers 
      
      
      if (b_cutDelta < b_deadband){
        analogOutput1 = VALVE_NEUTRAL;
        analogOutput2 = VALVE_NEUTRAL;
      }

      delta_previous_error = delta_error;
    }
    else
    {  
      analogOutput1 = VALVE_NEUTRAL;
      analogOutput2 = VALVE_NEUTRAL;
    }

    //Dac1.setVoltage(analogOutput1, false);
    //Dac2.setVoltage(analogOutput2, false);
    Output1_Handler();
    voltage = ((double)analogOutput1/4096) * 5.0;
    voltage2 =((double)analogOutput2/4096) * 5.0;
  }

  
  // ***************************************************************************************************************************************************
  // ****************************************************** OTHER FUNCTIONS*****************************************************************************
  // ***************************************************************************************************************************************************
  void printPgnAnnoucement(uint16_t _pgnNum, char* _pgnName, uint8_t _len, uint8_t _numParams)
  {
    Serial << "\r\n" << millis() << " OGX " << _pgnNum;
    Serial << " " << _pgnName << "(" << _len << ":" << _numParams << ") ";
  }

  void SendDataToOGX()
  {
    UDP.PGN_OGX.beginPacket(openGradeIP, OGX_PORT);   //Initiate transmission of data
    UDP.PGN_OGX.print(DATA_HEADER);
    UDP.PGN_OGX.print(",");
    UDP.PGN_OGX.print(b_autoState);
    UDP.PGN_OGX.print(",");
    UDP.PGN_OGX.print(voltage);
    UDP.PGN_OGX.print(",");
    UDP.PGN_OGX.print(voltage2);
    UDP.PGN_OGX.endPacket();  // Close communication
  }

  void loadFromEeprom()
  {
    // if using ESP, make sure to call EEPROM.begin(large enough for all EEPROM data); in your main setup()
    if (eeAddr < 0) return;

    uint8_t EEread;
    EEPROM.get(eeAddr + 0, EEread);        // read version identifier
    if (EEread != EE_ID) {              // check on first start and write EEPROM
      EEPROM.put(eeAddr + 0, EE_ID);
      EEPROM.put(eeAddr + 2, config);      // +2 to leave room for EE_IDENT
      Serial.print("\r\n\n* EEPROM version doesn't match, OpenGradeX config reset to default *");
    } else {
      EEPROM.get(eeAddr + 2, config);
      Serial.print("\r\n\nOpenGradeX config loaded from EEPROM");
    }
    setWorkingVars();
    if (debugLevel > 1) printConfig();
    #ifdef ESP32
      EEPROM.commit(); // needed for ESP
    #endif
  }

  void saveToEeprom()
  {
    if (eeAddr < 0) return;
    EEPROM.put(eeAddr + 2, config);
    #ifdef ESP32
      EEPROM.commit(); // needed for ESP
    #endif
    if (debugLevel > 1) Serial.print("\r\nOpenGradeX config saved to EEPROM");
  }

  void setWorkingVars()
  {
    Kp = double(config.Kp);
    Ki = double(config.Ki) / 100.0;
    Kd = double(config.Kd) * 100.0;

    retDeadband = VALVE_NEUTRAL - ((config.retDeadband / 200.0) * 4096);  // 10 / 200 * 4096 is the CNH/JD default SCV deadband of 0.25V 
    extDeadband = VALVE_NEUTRAL + ((config.extDeadband / 200.0) * 4096);

    retMin = RET_MIN[config.type] * 4096;
    extMax = EXT_MAX[config.type] * 4096;
  }

  void printConfig()
  {
    Serial << "\r\n- Kp: " << config.Kp << " (" << Kp << ")";
    Serial << "\r\n- Ki: " << config.Ki << " (" << Ki << ")";
    Serial << "\r\n- Kd: " << config.Kd << " (" << Kd << ")";
    Serial << "\r\n- Retract deadband: " << config.retDeadband;
    Serial << "\r\n- Extend  deadband: " << config.extDeadband;
    Serial << "\r\n- Valve type: " << config.type << " (" << valveNames[config.type] << ")";
  }


};
#endif