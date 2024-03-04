/*

  UDP - Receive PGNs sent from AgIO - sent to port 8888

  most of this code taken from old AIO I2C firmware

*/

#define UDP_MAX_PACKET_SIZE 40         // Buffer For Receiving 8888 UDP PGN Data
uint8_t udpData[UDP_MAX_PACKET_SIZE];  // UDP_TX_PACKET_MAX_SIZE is not large enough for machine pin settings PGN
uint32_t pgn254Time, pgn254MaxDelay, pgn254AveDelay, pgn254MinDelay = 99999;

void checkForPGNs()
{
  PGNusage.timeIn();

  if (UDP.isRunning) {                              // When ethernet is not running, return directly. parsePacket() will block when we don't
    uint16_t len = UDP.PGN.parsePacket();           //get data from AgIO sent by 9999 to this 8888

    if (UDP.PGN.remotePort() == 9999 && len > 4) {  //make sure from AgIO

      UDP.PGN.read(udpData, UDP_MAX_PACKET_SIZE);
      if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) {  // verify first 3 PGN header bytes

        if (udpData[3] == 0xFE && len == 14)        // 0xFE (254) - Steer Data (sent at GPS freq, ie 10hz (100ms))
        {
          //Serial << "\r\nSteer Data 0xFE (254), " << len << " bytes";
          //Serial.print("\rsteerData update period: "); Serial.println(gpsSpeedUpdateTimer);
          /*Serial.printf(" %6i", micros() - pgn254Time);
          pgn254Time = micros();
          uint32_t pgn254Delay = pgn254Time - nmeaPgnSendTime;
          if (pgn254Delay < pgn254MinDelay) pgn254MinDelay = pgn254Delay;
          if (pgn254Delay > pgn254MaxDelay) pgn254MaxDelay = pgn254Delay;
          if (pgn254AveDelay == 0) pgn254AveDelay = pgn254Delay;
          else pgn254AveDelay = pgn254AveDelay * 0.99 + pgn254Delay * 0.01;
          Serial.printf("->PGN254 delay: %4iuS  %4i %4i %4i", pgn254Delay, pgn254MinDelay, pgn254AveDelay, pgn254MaxDelay);*/
          float gpsSpeed = ((float)(udpData[5] | udpData[6] << 8)) * 0.1;   // speed data comes in as km/hr x10
          //Serial << "\r\n speed:" << gpsSpeed << " "; Serial.print(udpData[5], BIN); Serial << " "; Serial.print(udpData[6], BIN);
          speedPulse.updateSpeed(gpsSpeed);

          prevGuidanceStatus = guidanceStatus;
          guidanceStatus = udpData[7];
          guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

          //Bit 8,9    set point steer angle * 100 is sent
          steerAngleSetPoint = ((float)(udpData[8] | ((int8_t)udpData[9]) << 8)) * 0.01;  //high low bytes

          // moved to 100hz AS loop for quicker reactions
          if ((bitRead(guidanceStatus, 0) == 0) || (steerState == 0)) { // || (gpsSpeed < 0.1)) {
            watchdogTimer = WATCHDOG_FORCE_VALUE;  //turn off steering motor
            //Serial.print(" OFF");
          } else {                                 //valid conditions to turn on autosteer
            watchdogTimer = 0;                     //reset watchdog
            //Serial.print(" ON");
          }

          //Bit 10 Tram
          //tram = udpData[10];

          //Bit 11
          //relay = udpData[11];

          //Bit 12
          //relayHi = udpData[12];

          //----------------------------------------------------------------------------
          // Reply to send to AgIO
          // fromAutoSteerData FD 253 - ActualSteerAngle*100 -56, SwitchByte-7, pwmDisplay-8
          uint8_t PGN_253[] = { 0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

          int16_t sa = (int16_t)(steerAngleActual * 100);
          PGN_253[5] = (uint8_t)sa;
          PGN_253[6] = sa >> 8;

          // heading
          PGN_253[7] = (uint8_t)9999;
          PGN_253[8] = 9999 >> 8;

          // roll
          PGN_253[9] = (uint8_t)8888;
          PGN_253[10] = 8888 >> 8;

          PGN_253[11] = switchByte;
          PGN_253[12] = (uint8_t)pwmDrive;

          //checksum
          int16_t CK_A = 0;
          for (uint8_t i = 2; i < sizeof(PGN_253) - 1; i++)
            CK_A = (CK_A + PGN_253[i]);

          PGN_253[sizeof(PGN_253) - 1] = CK_A;

          //off to AOG
          UDP_Susage.timeIn();
          UDP.SendUdpByte(PGN_253, sizeof(PGN_253), UDP.broadcastIP, UDP.portAgIO_9999);
          UDP_Susage.timeOut();

          //Steer Data 2 -------------------------------------------------
          if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
            if (aog2Count++ > 2) {
              // fromAutoSteerData FD 250 - sensor values etc
              uint8_t PGN_250[] = { 0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

              //Send fromAutosteer2
              PGN_250[5] = (byte)sensorReading;

              //add the checksum for AOG2
              CK_A = 0;

              for (uint8_t i = 2; i < sizeof(PGN_250) - 1; i++) {
                CK_A = (CK_A + PGN_250[i]);
              }

              PGN_250[sizeof(PGN_250) - 1] = CK_A;

              //off to AOG
              UDP_Susage.timeIn();
              UDP.SendUdpByte(PGN_250, sizeof(PGN_250), UDP.broadcastIP, UDP.portAgIO_9999);
              UDP_Susage.timeOut();
              aog2Count = 0;
            }
          }
        }  // 0xFE (254) - Steer Data


        else if (udpData[3] == 0xFC && len == 14)         // 0xFC (252) - Steer Settings
        {
          Serial << "\r\nSteer Settings 0xFC (252), " << len << " bytes";
          //PID values
          steerSettings.Kp = ((float)udpData[5]);    // read Kp from AgOpenGPS
          steerSettings.highPWM = udpData[6];        // read high pwm
          steerSettings.lowPWM = (float)udpData[7];  // read lowPWM from AgOpenGPS
          steerSettings.minPWM = udpData[8];         //read the minimum amount of PWM for instant on

          float temp = (float)steerSettings.minPWM * 1.2;
          steerSettings.lowPWM = (byte)temp;

          steerSettings.steerSensorCounts = udpData[9];   //sent as setting displayed in AOG
          steerSettings.wasOffset = (udpData[10]);        //read was zero offset Lo
          steerSettings.wasOffset |= (udpData[11] << 8);  //read was zero offset Hi
          steerSettings.AckermanFix = (float)udpData[12] * 0.01;

          //Serial << "\r\nwasOffset: " << steerSettings.wasOffset;

          EEPROM.put(10, steerSettings);
          steerSettingsInit();  // Re-Init steer settings
        }  // 0xFC (252) - Steer Settings


        else if (udpData[3] == 0xFB && len == 14)  // 0xFB (251) - SteerConfig
        {
          Serial << "\r\nSteer Config, " << len << " bytes";
          uint8_t sett = udpData[5]; //setting0
          if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
          if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
          if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
          if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
          if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
          if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
          if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
          if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

          steerConfig.PulseCountMax = udpData[6];

          sett = udpData[8]; //setting1 - Danfoss valve etc
          if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
          if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
          if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
          if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

          EEPROM.put(40, steerConfig);            
          steerConfigInit();  // Re-Init

        }  // 0xFB (251) - SteerConfig


        else if (udpData[3] == 200 && len == 9)  // 0xC8 (200) - Hello from AgIO
        {
          //Serial << "\r\nHello from AgIO 0xC8 (200), " << len << " bytes";

          // reply as Steer Module
          uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
          if (autoSteerEnabled) {
            int16_t sa = (int16_t)(steerAngleActual * 100);

            helloFromAutoSteer[5] = (uint8_t)sa;
            helloFromAutoSteer[6] = sa >> 8;

            uint16_t helloSteerPosition = steeringPosition - 6800;
            helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
            helloFromAutoSteer[8] = helloSteerPosition >> 8;
            helloFromAutoSteer[9] = switchByte;

            UDP_Susage.timeIn();
            UDP.SendUdpByte(helloFromAutoSteer, sizeof(helloFromAutoSteer), UDP.broadcastIP, UDP.portAgIO_9999);
            UDP_Susage.timeOut();
          }

          #ifdef MACHINE_H
            if (machine.isInit) {
              uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };
              UDP_Susage.timeIn();
              UDP.SendUdpByte(helloFromMachine, sizeof(helloFromMachine), UDP.broadcastIP, UDP.portAgIO_9999);
              UDP_Susage.timeOut();
            }
          #endif
              
          // reply as IMU if equipped
          if (useBNO08xRVC) {
            uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
            UDP_Susage.timeIn();
            UDP.SendUdpByte(helloFromIMU, sizeof(helloFromIMU), UDP.broadcastIP, UDP.portAgIO_9999);
            UDP_Susage.timeOut();
          }
        }  // 0xC8 (200) - Hello from AgIO


        else if (udpData[3] == 201 && len == 11)  // 0xC9 (201) - Subnet Change
        {
          Serial << "\r\nSubnet Change 0xC9 (201), " << len << " bytes";
          if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)  //save in EEPROM and restart
          {
            Serial << "\r\n- IP changed from " << UDP.myIP;
            UDP.myIP[0] = udpData[7];
            UDP.myIP[1] = udpData[8];
            UDP.myIP[2] = udpData[9];

            Serial << " to " << UDP.myIP;
            Serial << "\r\n- Saving to EEPROM and restarting Teensy";

            UDP.SaveModuleIP();  //save in EEPROM and restart
            delay(10);
            SCB_AIRCR = 0x05FA0004;  //Teensy Reset
          }
        }  // 0xC9 (201) - Subnet Change


        else if (udpData[3] == 202 && len == 9)    // 0xCA (202) - Scan Request
        {
          Serial << "\r\nScan Request 0xCA (202), " << len << " bytes";
          if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202) {
            IPAddress rem_ip = UDP.PGN.remoteIP();
            IPAddress ipDest = { 255, 255, 255, 255 };

            uint8_t scanReplySteer[] = { 128, 129, 126, 203, 7,
                                    UDP.myIP[0], UDP.myIP[1], UDP.myIP[2], UDP.myIP[3],
                                    rem_ip[0], rem_ip[1], rem_ip[2], 23 };
            //checksum
            int16_t CK_A = 0;
            for (uint8_t i = 2; i < sizeof(scanReplySteer) - 1; i++) {
              CK_A = (CK_A + scanReplySteer[i]);
            }
            scanReplySteer[sizeof(scanReplySteer) - 1] = CK_A;
            UDP_Susage.timeIn();
            UDP.SendUdpByte(scanReplySteer, sizeof(scanReplySteer), ipDest, UDP.portAgIO_9999);
            UDP_Susage.timeOut();

            if (useBNO08xRVC) {
              uint8_t scanReplyIMU[] = { 128, 129, 121, 203, 7,
                                      UDP.myIP[0], UDP.myIP[1], UDP.myIP[2], UDP.myIP[3],
                                      rem_ip[0], rem_ip[1], rem_ip[2], 23 };
              //checksum
              CK_A = 0;
              for (uint8_t i = 2; i < sizeof(scanReplyIMU) - 1; i++) {
                CK_A = (CK_A + scanReplyIMU[i]);
              }
              scanReplyIMU[sizeof(scanReplyIMU) - 1] = CK_A;
              UDP_Susage.timeIn();
              UDP.SendUdpByte(scanReplyIMU, sizeof(scanReplyIMU), ipDest, UDP.portAgIO_9999);
              UDP_Susage.timeOut();
            }

            #ifdef MACHINE_H
            if (machine.isInit) {
              uint8_t scanReplyMachine[] = { 128, 129, 123, 203, 7,
                                      UDP.myIP[0], UDP.myIP[1], UDP.myIP[2], UDP.myIP[3],
                                      rem_ip[0], rem_ip[1], rem_ip[2], 23 };
              //checksum
              CK_A = 0;
              for (uint8_t i = 2; i < sizeof(scanReplyMachine) - 1; i++) {
                CK_A = (CK_A + scanReplyMachine[i]);
              }
              scanReplyMachine[sizeof(scanReplyMachine) - 1] = CK_A;
              UDP_Susage.timeIn();
              UDP.SendUdpByte(scanReplyMachine, sizeof(scanReplyMachine), ipDest, UDP.portAgIO_9999);
              UDP_Susage.timeOut();
            }
            #endif

            Serial.printf("\r\n ---------\r\n%s\r\nCPU Temp:%.1f CPU Speed:%iMhz GPS Baud:%i", inoVersion, tempmonGetTemp(), F_CPU_ACTUAL / 1000000, baudGPS);
            Serial.print("\r\nAgIO IP:   "); Serial.print(rem_ip);
            Serial.print("\r\nModule IP: "); Serial.print(UDP.myIP);

            /*if (!Autosteer_running) Serial.println("\r\n!! Autosteer disabled... Check ADS1115");
            else if (PWM_Frequency == 0) Serial.println("\r\nAutosteer running, PWM Frequency = 490hz");
            else if (PWM_Frequency == 1) Serial.println("\r\nAutosteer running, PWM Frequency = 122hz");
            else if (PWM_Frequency == 2) Serial.println("\r\nAutosteer running, PWM Frequency = 3921hz");*/

            if (useBNO08xRVC) Serial.print("\r\nBNO08x available via Serial/RVC Mode");
            else Serial.print("\r\n* No IMU available *");

            /*if (GGA_Available == false) Serial.println("\r\n!! GPS Data Missing... Check F9P Config");
            else if (!useDual) Serial.println("\r\nGPS Single GPS mode");
            else if (useDual && !dualDataFail && !dualRTKFail && !dualBaselineFail) Serial.println("\r\nGPS Dual GPS mode");
            else if (dualDataFail) Serial.println("\r\n!! Dual Data Checksum Failed");
            else if (dualRTKFail) Serial.println("\r\n!! Dual RTK/Quality Failed... Check Antennas");
            else if (dualBaselineFail) Serial.println("\r\n!! Dual Baseline Moving Too Much... Check Antennas");*/
            
            Serial.println("\r\r\n ---------");
          }
        }  // 0xCA (202) - Scan Request

       
        else    // catch all other UDP PGN data
        {
          bool none = true;
          #ifdef AIOv50a
            PGNusage.timeOut();
            MACHusage.timeIn();
            if (machine.parsePGN(udpData, len))    // look for Machine PGNs
            {
              none = false;
              //Serial << "\r\nChecking machine PGNs";
              // 0xE5 (229) - 64 Section Data
              // 0xEB (235) - Section Dimensions
              // 0xEC (236) - Machine Pin Config
              // 0xEE (238) - Machine Config
              // 0xEF (239) - Machine Data
            }
            MACHusage.timeOut();
            PGNusage.timeIn();
          #endif

          if (none) {
            Serial.print("\r\n0x"); Serial.print(udpData[3], HEX); Serial.print("("); Serial.print(udpData[3]);
            Serial.print(") - Unknown PGN data, len: "); Serial.print(len);
          }
        }

      } // end 0x80 0x81 0x7F
    }
  }
  PGNusage.timeOut();
}


/*
* To receive NMEA sent via UDP from GPS Source
   - listen on port 2211 GPS1 (single/dual)
   - additional related port number 2222 GPS2 (dual?)
   - additional related port number 2233 RTCM
* Character data
*/
char NMEA_packetBuffer[256];       // buffer for receiving GGA and VTG

void udpNMEA() {
  if (!UDP.isRunning) return;  // When ethernet is not running, return directly. parsePacket() will block when we don't

  int packetLength = UDP.NMEA.parsePacket();
  if (packetLength > 0) {
    UDP.NMEA.read(NMEA_packetBuffer, packetLength);
    for (int i = 0; i < packetLength; i++) {
      nmeaParser << NMEA_packetBuffer[i];
    }
  }
}


/*
*  To receive RTCM sent via UDP from AgIO NTRIP client - listen on port 2233 RTCM
*/
char RTCM_packetBuffer[buffer_size];
uint32_t ntripUpdateTime, ntripCheckTime;

void udpNtrip() {
  NTRIPusage.timeIn();

  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (UDP.isRunning) {
    if (millis() > ntripCheckTime) {  // limit update rate to save cpu time
    
      unsigned int packetLength = UDP.RTCM.parsePacket(); // this uses most of the cpu time in this function unless SerialGPS has low baud
      ntripCheckTime = millis() + 10;

      if (packetLength > 0) {
        //Serial.print("\r\nNTRIP "); Serial.print(millis() - ntripUpdateTime); Serial.print(" len:"); Serial.print(packetLength);
        UDP.RTCM.read(RTCM_packetBuffer, buffer_size);
        SerialGPS->write(RTCM_packetBuffer, buffer_size);

        // up to 256 byte packets are sent from AgIO and most NTRIP RTCM updates are larger so there's usually two packets per update
        if (packetLength < buffer_size){    // if buffer was not full, then end of NTRIP packet, can wait 800ms until we start checking again
          ntripCheckTime = millis() + 800;  // most base stations send updates every 1000ms
        }
          
        ntripUpdateTime = millis();
      }
    }
  }
  NTRIPusage.timeOut();
}
