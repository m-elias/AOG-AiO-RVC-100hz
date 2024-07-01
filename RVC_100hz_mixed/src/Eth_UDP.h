//#ifndef _ETHER_h
//#define _ETHER_h

#include <stdint.h>
#include "elapsedMillis.h"
#include "IPAddress.h"
#include "Arduino.h"
#include "AsyncUDP_Teensy41.h"
#include <EEPROM.h>

#define UDP_MAX_PACKET_SIZE 40         // Buffer For Receiving 8888 UDP PGN Data

AsyncUDP GNSS; // UDP object for incoming NMEA
AsyncUDP RTCM; // UDP object for incoming RTCM
AsyncUDP PGN;  // UDP object for in/out PGN

class Eth_UDP
{
public:
	IPAddress myIP = { 192, 168, 5, 126 };  // 126 default IP for steer module
  IPAddress myNetmask = {255, 255, 255, 0};
  IPAddress myGW = {192, 168, 5, 1};
  IPAddress mydnsServer= {192, 168, 5, 1};
  IPAddress broadcastIP;
  byte mac[6] = { 0x0A, 0x0F, myIP[0], myIP[1], myIP[2], myIP[3] };     // create unique MAC from IP as IP should already be unique

	// This modules listens to GPS sent on (carry over from Ace)
  // likely not needed but may be convenient for simulating a GPS receiver on the bench using UDP
	unsigned int portGNSS_2211 = 2211;     // Why 2211? 22XX=GPS then 2211=GPS1 2222=GPS2 2233=RTCM3 corrections easy to remember.              

	unsigned int portRTCM_2233 = 2233;     // Why 2211? 22XX=GPS then 2211=GPS1 2222=GPS2 2233=RTCM3 corrections easy to remember.                 
  
	unsigned int portSteer_8888 = 8888;    // UDP port that Modules (like this one) listen to
	//EthernetUDP PGN;                       // UDP object for PGNs on port 8888

	unsigned int portAgIO_9999 = 9999;     // UDP port that AgIO listens to, send data here

	bool isRunning = false;                // set true with successful Eth Start()
  int8_t linkStatus = -1;                // 0 - Unknown, 1 - LinkON, 2 - LinkOFF
	const int EE_ver = 2402;               // if value in eeprom does not match, overwrite with defaults

  elapsedMillis initTimer = 2000;

  void Eth_EEPROM() {
    Serial.println();
    Serial.println("EEPROM IP Address reading");
    uint16_t eth_ee_read;
    EEPROM.get(60, eth_ee_read);

    if (eth_ee_read != EE_ver) {     // if EE is out of sync, write defaults to EE
      EEPROM.put(60, EE_ver);
      SaveModuleIP();
      Serial.print("\r\n\nWriting Eth defaults to EEPROM\r\n");
    } else {
      EEPROM.get(62, myIP[0]);
      EEPROM.get(63, myIP[1]);
      EEPROM.get(64, myIP[2]);
      mac[2] = myIP[0];
      mac[3] = myIP[1];
      mac[4] = myIP[2];
      Serial.println("EEPROM IP Address reading step 1");
    }

    broadcastIP[0] = myIP[0];
    broadcastIP[1] = myIP[1];
    broadcastIP[2] = myIP[2];
    broadcastIP[3] = 255;                // same subnet as module's IP but use broadcast
    Serial.println("EEPROM IP Address reading Step 2");
    Serial.println(broadcastIP);
    Serial.print("mac: ");
    for(int i=0; i<sizeof(mac); i++){
    printHex(mac[i]);
    }
    Serial.println();
  }

  bool init()
  {
    Serial.println("Eth_UDP init");
    if (isRunning) return true;
    //Ethernet.MACAddress(mac);                 // get Teensy's internal MAC, doesn't work reliably
    //Ethernet.begin(mac, 2000, 2000);          // start dhcp connection with 2s timeout, that's enough time to get an eth linkStatus update
    //Ethernet.begin(mac, myIP);                // blocks if unplugged
    Ethernet.setDHCPEnabled(false);             // Must be set to false if using non-blocking begin() or DHCP client will wipe out static settings in 6 minutes killing the ethernet connection.
    Ethernet.begin(mac, 0);                     // non-blocking method, set IP later

    // Check for Ethernet hardware present, always returns "EthernetW5500" (3) for Teensy 4.1 w/Eth
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("\r\n\n*** Ethernet was not found. GPS via USB only ***");   // maybe using non Ethernet Teensy?
      return false;
    }

    Ethernet.setLocalIP(myIP);                  // also non-blocking as opposed to Ethernet.begin(mac, myIP) which block with unplugged/unconnected cable
    Ethernet.setSubnetMask(myNetmask);
    Ethernet.setGatewayIP(myGW);
    Ethernet.setDNSServerIP(mydnsServer);
    Serial.print("\r\n\nEthernet connection set with static IP address");

    Serial.print("\r\n- Using MAC address: ");
    for (byte octet = 0; octet < 6; octet++) {
      if (mac[octet] < 0x10) Serial.print("0");
      Serial.print(mac[octet], HEX);
      if (octet < 5) Serial.print(':');
    }

    Serial.print("\r\n- Ethernet IP of module: ");
    Serial.print(Ethernet.localIP());

    Serial.print("\r\n- Ethernet Broadcast IP: ");
    Serial.print(broadcastIP);

    Serial.print("\r\n\nSetting up UDP comms");
    Serial.print("\r\n- Sending to AgIO port: ");
    Serial.print(portAgIO_9999);

    // init UPD Port getting AutoSteer (8888) data from AGIO
    // if (PGN.(portSteer_8888)) {
    //   Serial.print("\r\n- Ethernet UDP PGN listening to port: ");
    //   Serial.print(portSteer_8888);
    // }

  if (GNSS.listen(portGNSS_2211)) {
        Serial.print("\r\nNMEA UDP Listening on: "); Serial.print(Ethernet.localIP());
        Serial.print(":"); Serial.print(portGNSS_2211);

        // this function is triggered asynchronously(?) by the AsyncUDP library
        GNSS.onPacket([&](AsyncUDPPacket packet) {
          gNSS(packet);          
        }); // all the brackets and ending ; are necessary!
      }

  if (RTCM.listen(portRTCM_2233)) {
        Serial.print("\r\nRTCM UDP Listening on: "); Serial.print(Ethernet.localIP());
        Serial.print(":"); Serial.print(portRTCM_2233);

        // this function is triggered asynchronously(?) by the AsyncUDP library
        RTCM.onPacket([&](AsyncUDPPacket packet) {
          nTrip(packet);
        }); // all the brackets and ending ; are necessary!
      }

  if (PGN.listen(portSteer_8888)) {
        Serial.print("\r\nRTCM UDP Listening on: "); Serial.print(Ethernet.localIP());
        Serial.print(":"); Serial.print(portSteer_8888);

        // this function is triggered asynchronously(?) by the AsyncUDP library
        PGN.onPacket([&](AsyncUDPPacket packet) {
          checkForPGNs(packet);
        }); // all the brackets and ending ; are necessary!
      }

    isRunning = true;
    return true;
  }

void checkForPGNs(AsyncUDPPacket packet)
{
  Serial.println("Check for PGNs **********************************************");
  #ifdef AIOv50a
  ESP32usage.timeIn();
  if (SerialESP32->available())
  {
    static uint8_t incomingBytes[50];
    static uint8_t incomingIndex;
    incomingBytes[incomingIndex] = SerialESP32->read();
    incomingIndex++;
    /*Serial.print("\r\nindex: "); Serial.print(incomingIndex);
    Serial.print(" ");
    for (byte i = 0; i < incomingIndex; i++) {
      Serial.print(incomingBytes[i]);
      Serial.print(" ");
    }*/

    if (incomingBytes[incomingIndex - 2] == 13 && incomingBytes[incomingIndex - 1] == 10)
    {
      if (incomingBytes[0] == 128 && incomingBytes[1] == 129)
      {

        // Modules--Wifi:9999-->ESP32--serial-->Teensy--ethernet:9999-->AgIO
        //SendUdpByte(incomingBytes, incomingIndex - 2, broadcastIP, portAgIO_9999);

        //pass data to USB for debug
        /*Serial.print("\r\nE32-s->T41-e:9999->AgIO ");
        for (byte i = 0; i < incomingIndex - 2; i++) {
          Serial.print(incomingBytes[i]);
          Serial.print(" ");
        }
        Serial.print((String)" (" + SerialESP32->available() + ")");
        */
      } else {
        Serial.print("\r\n\nCR/LF detected but [0]/[1] bytes != 128/129\r\n");
      }
      incomingIndex = 0;
    }
  }
  ESP32usage.timeOut();
  #endif  // AIOv50a


  PGNusage.timeIn();
  static uint32_t pgnCheckTime;
  uint32_t millisNow = millis();
  // if (millisNow < pgnCheckTime) return;   // only need to check for new PGN data every ms, not 100s of times per ms
  // //Serial.print((String)"\r\n" + millisNow + " PGN check " + pgnCheckTime);
  // pgnCheckTime = millisNow + 1;     // allow check every ms

  if (!isRunning) return;                           // When ethernet is not running, return directly. parsePacket() will block when we don't
  // uint16_t packet.length() = PGN.parsePacket();                 //get data from AgIO sent by 9999 to this 8888
  // if (PGN.remotePort() != 9999 || packet.length() < 5) return;  //make sure from AgIO

  // uint8_t packet.data()[UDP_MAX_PACKET_SIZE];  // UDP_TX_PACKET_MAX_SIZE is not large enough for machine pin settings PGN
  // PGN.read(packet.data(), UDP_MAX_PACKET_SIZE);

  if (packet.data()[0] != 0x80 || packet.data()[1] != 0x81 || packet.data()[2] != 0x7F) return;  // verify first 3 PGN header bytes
  bool pgnMatched = false;

  #ifdef AIOv50a
  if (packet.data()[3] != 100) {
    ESP32usage.timeIn();
    SerialESP32->write(packet.data(), packet.length());
    SerialESP32->println();   // to signal end of PGN
    /*Serial.print("\r\nAgIO-e:8888->T41-s->E32 ");
    for (uint8_t i = 0; i < packet.length(); i++) {
      Serial.print(packet.data()[i]); Serial.print(" ");
    }*/
    ESP32usage.timeOut();
  }
  #endif

  // changed to multiple IF statements instead of IF ELSE so that AgIO Hello and Scan Request PGNs can be pickedup by other object/classes (ie machine)

  if (packet.data()[3] == 100 && packet.length() == 22)  // 0x64 (100) - Corrected Position
  {
    //printPgnAnnoucement(packet.data()[3], (char*)"Corrected Position", packet.length());
    /*
    union {           // both variables in the union share the same memory space
      byte array[8];  // fill "array" from an 8 byte array converted in AOG from the "double" precision number we want to send
      double number;  // and the double "number" has the original "double" precision number from AOG
    } lat, lon;

    for (byte i = 0; i < 8; i++)
    {
      lon.array[i] = packet.data()[i+5];
      lat.array[i] = packet.data()[i+13];
    }*/
    /*Serial.print("\r\n");
    Serial.print(lat.number, 13);
    Serial.print(" ");
    Serial.print(lon.number, 13);*/

    //buildNMEA(lat.number, lon.number);

    return;                    // no other processing needed
  }


  if (packet.data()[3] == 200 && packet.length() == 9)  // 0xC8 (200) - Hello from AgIO
  {
    //printPgnAnnoucement(packet.data()[3], (char*)"Hello from AgIO", packet.length());
    LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::AGIO_CONNECTED, true);

    //Serial.print("\r\n***** AgIO Hello byte 5-7: "); Serial.print(packet.data()[5]); Serial.print(" "); Serial.print(packet.data()[6]); Serial.print(" "); Serial.print(packet.data()[7]); Serial.print(" ");

    // reply as Steer Module
    uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
    if (autoSteerEnabled) {
      int16_t sa = (int16_t)(steerAngleActual * 100);

      helloFromAutoSteer[5] = (uint8_t)sa;
      helloFromAutoSteer[6] = sa >> 8;

      uint16_t helloSteerPosition = steeringPosition; // - 6800; steeringPosition is already centered & offset in Autosteer.ino
      helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
      helloFromAutoSteer[8] = helloSteerPosition >> 8;
      helloFromAutoSteer[9] = switchByte;

      UDP_Susage.timeIn();
      SendUdpByte(helloFromAutoSteer, sizeof(helloFromAutoSteer), broadcastIP, portAgIO_9999);
      UDP_Susage.timeOut();
    }

    // reply as IMU if equipped
    if (BNO.isActive) {
      uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
      UDP_Susage.timeIn();
      SendUdpByte(helloFromIMU, sizeof(helloFromIMU), broadcastIP, portAgIO_9999);
      UDP_Susage.timeOut();
    }

    #ifdef MACHINE_H
      if (machine.isInit) {
        uint8_t helloFromMachine[] = { 0x80, 0x81, 123, 123, 5, 0, 0, 0, 0, 0, 71 };
        helloFromMachine[5] = B10101010;  // should be changed to read actual machine output states
        helloFromMachine[6] = B01010101;
        UDP_Susage.timeIn();
        SendUdpByte(helloFromMachine, sizeof(helloFromMachine), broadcastIP, portAgIO_9999);
        UDP_Susage.timeOut();
      }
    #endif
        
    pgnMatched = true;
    //return;         // no return, allow machine object to process machine reply below
  } // 0xC8 (200) - Hello from AgIO


  if (packet.data()[3] == 201 && packet.length() == 11)  // 0xC9 (201) - Subnet Change
  {
    printPgnAnnoucement(packet.data()[3], (char*)"Subnet Change", packet.length());
    if (packet.data()[4] == 5 && packet.data()[5] == 201 && packet.data()[6] == 201)  //save in EEPROM and restart
    {
      Serial << "\r\n- IP changed from " << myIP;
      myIP[0] = packet.data()[7];
      myIP[1] = packet.data()[8];
      myIP[2] = packet.data()[9];

      Serial << " to " << myIP;
      Serial << "\r\n- Saving to EEPROM and restarting Teensy";

      SaveModuleIP();  //save in EEPROM and restart
      delay(10);
      SCB_AIRCR = 0x05FA0004;  //Teensy Reset
    }
    return;                    // no other processing needed
  }  // 0xC9 (201) - Subnet Change


  if (packet.data()[3] == 202 && packet.length() == 9)    // 0xCA (202) - Scan Request
  {
    printPgnAnnoucement(packet.data()[3], (char*)"Scan Request", packet.length());
    if (packet.data()[4] == 3 && packet.data()[5] == 202 && packet.data()[6] == 202) {
      IPAddress rem_ip = packet.remoteIP();
      IPAddress ipDest = { 255, 255, 255, 255 };

      uint8_t scanReplySteer[] = { 128, 129, 126, 203, 7,
                              myIP[0], myIP[1], myIP[2], myIP[3],
                              rem_ip[0], rem_ip[1], rem_ip[2], 23 };
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < sizeof(scanReplySteer) - 1; i++) {
        CK_A = (CK_A + scanReplySteer[i]);
      }
      scanReplySteer[sizeof(scanReplySteer) - 1] = CK_A;
      UDP_Susage.timeIn();
      SendUdpByte(scanReplySteer, sizeof(scanReplySteer), ipDest, portAgIO_9999);
      UDP_Susage.timeOut();

      if (BNO.isActive) {
        uint8_t scanReplyIMU[] = { 128, 129, 121, 203, 7,
                                myIP[0], myIP[1], myIP[2], myIP[3],
                                rem_ip[0], rem_ip[1], rem_ip[2], 23 };
        CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReplyIMU) - 1; i++) {
          CK_A = (CK_A + scanReplyIMU[i]);
        }
        scanReplyIMU[sizeof(scanReplyIMU) - 1] = CK_A;
        UDP_Susage.timeIn();
        SendUdpByte(scanReplyIMU, sizeof(scanReplyIMU), ipDest, portAgIO_9999);
        UDP_Susage.timeOut();
      }

      #ifdef MACHINE_H
      if (machine.isInit) {
        uint8_t scanReplyMachine[] = { 128, 129, 123, 203, 7,
                                myIP[0], myIP[1], myIP[2], myIP[3],
                                rem_ip[0], rem_ip[1], rem_ip[2], 23 };
        CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReplyMachine) - 1; i++) {
          CK_A = (CK_A + scanReplyMachine[i]);
        }
        scanReplyMachine[sizeof(scanReplyMachine) - 1] = CK_A;
        UDP_Susage.timeIn();
        SendUdpByte(scanReplyMachine, sizeof(scanReplyMachine), ipDest, portAgIO_9999);
        UDP_Susage.timeOut();
      }
      #endif

      Serial.printf("\r\n ---------\r\n%s\r\nCPU Temp:%.1f CPU Speed:%iMhz GPS Baud:%i", inoVersion, tempmonGetTemp(), F_CPU_ACTUAL / 1000000, baudGPS);
      Serial.print("\r\nAgIO IP:   "); Serial.print(rem_ip);
      Serial.print("\r\nModule IP: "); Serial.print(myIP);

      /*if (!Autosteer_running) Serial.println("\r\n!! Autosteer disabled... Check ADS1115");
      else if (PWM_Frequency == 0) Serial.println("\r\nAutosteer running, PWM Frequency = 490hz");
      else if (PWM_Frequency == 1) Serial.println("\r\nAutosteer running, PWM Frequency = 122hz");
      else if (PWM_Frequency == 2) Serial.println("\r\nAutosteer running, PWM Frequency = 3921hz");*/

      if (BNO.isActive) Serial.print("\r\nBNO08x available via Serial/RVC Mode");
      else Serial.print("\r\n* No IMU available *");

      /*if (GGA_Available == false) Serial.println("\r\n!! GPS Data Missing... Check F9P Config");
      else if (!useDual) Serial.println("\r\nGPS Single GPS mode");
      else if (useDual && !dualDataFail && !dualRTKFail && !dualBaselineFail) Serial.println("\r\nGPS Dual GPS mode");
      else if (dualDataFail) Serial.println("\r\n!! Dual Data Checksum Failed");
      else if (dualRTKFail) Serial.println("\r\n!! Dual RTK/Quality Failed... Check Antennas");
      else if (dualBaselineFail) Serial.println("\r\n!! Dual Baseline Moving Too Much... Check Antennas");*/
      
      Serial.println("\r\r\n ---------");
    }
    return;
  } // 0xCA (202) - Scan Request


  if (packet.data()[3] == 0xFB && packet.length() == 14)  // 0xFB (251) - SteerConfig
  {
    printPgnAnnoucement(packet.data()[3], (char*)"Steer Config", packet.length());
    uint8_t sett = packet.data()[5]; //setting0
    if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
    if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
    if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
    if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
    if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
    if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
    if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
    if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

    steerConfig.PulseCountMax = packet.data()[6];
    steerConfig.MinSpeed = packet.data()[7];

    sett = packet.data()[8]; //setting1 - Danfoss valve etc
    if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
    if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
    if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
    if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

    Serial.print("\r\nInvertWAS "); Serial.print(steerConfig.InvertWAS);
    Serial.print("\r\nIsRelayActiveHigh "); Serial.print(steerConfig.IsRelayActiveHigh);
    Serial.print("\r\nMotorDriveDirection "); Serial.print(steerConfig.MotorDriveDirection);
    Serial.print("\r\nSingleInputWAS "); Serial.print(steerConfig.SingleInputWAS);
    Serial.print("\r\nCytronDriver "); Serial.print(steerConfig.CytronDriver);
    Serial.print("\r\nSteerSwitch "); Serial.print(steerConfig.SteerSwitch);
    Serial.print("\r\nSteerButton "); Serial.print(steerConfig.SteerButton);
    Serial.print("\r\nShaftEncoder "); Serial.print(steerConfig.ShaftEncoder);
    Serial.print("\r\nIsDanfoss "); Serial.print(steerConfig.IsDanfoss);
    Serial.print("\r\nPressureSensor "); Serial.print(steerConfig.PressureSensor);
    Serial.print("\r\nCurrentSensor "); Serial.print(steerConfig.CurrentSensor);
    Serial.print("\r\nIsUseY_Axis "); Serial.print(steerConfig.IsUseY_Axis);
    Serial.print("\r\nPulseCountMax "); Serial.print(steerConfig.PulseCountMax);
    Serial.print("\r\nMinSpeed "); Serial.print(steerConfig.MinSpeed);
    Serial.println();

    EEPROM.put(40, steerConfig);            
    steerConfigInit();  // Re-Init
    return;             // no other processing needed
  }  // 0xFB (251) - SteerConfig


  if (packet.data()[3] == 0xFC && packet.length() == 14)         // 0xFC (252) - Steer Settings
  {
    printPgnAnnoucement(packet.data()[3], (char*)"Steer Settings", packet.length());
    //PID values
    steerSettings.Kp = ((float)packet.data()[5]);    // read Kp from AgOpenGPS
    steerSettings.highPWM = packet.data()[6];        // read high pwm
    steerSettings.lowPWM = (float)packet.data()[7];  // read lowPWM from AgOpenGPS
    steerSettings.minPWM = packet.data()[8];         // read the minimum amount of PWM for instant on

    float temp = (float)steerSettings.minPWM * 1.2;
    steerSettings.lowPWM = (byte)temp;

    steerSettings.steerSensorCounts = packet.data()[9];   // sent as setting displayed in AOG
    
    //steerSettings.wasOffset = (packet.data()[10]);        // read was zero offset Lo
    //steerSettings.wasOffset |= (packet.data()[11] << 8);  // read was zero offset Hi
    int16_t newWasOffset = (packet.data()[10]);        // read was zero offset Lo
    newWasOffset        |= (packet.data()[11] << 8);   // read was zero offset Hi

    #ifdef JD_DAC_H
      jdDac.setMaxPWM(steerSettings.highPWM);
      if (newWasOffset != steerSettings.wasOffset) {
        jdDac.centerDac();
      }
    #endif

    steerSettings.wasOffset = newWasOffset;
    steerSettings.AckermanFix = (float)packet.data()[12] * 0.01;

    Serial.print("\r\n Kp "); Serial.print(steerSettings.Kp);
    Serial.print("\r\n highPWM "); Serial.print(steerSettings.highPWM);
    Serial.print("\r\n lowPWM "); Serial.print(steerSettings.lowPWM);
    Serial.print("\r\n minPWM "); Serial.print(steerSettings.minPWM);
    Serial.print("\r\n steerSensorCounts "); Serial.print(steerSettings.steerSensorCounts);
    Serial.print("\r\n wasOffset "); Serial.print(steerSettings.wasOffset);
    Serial.print("\r\n AckermanFix "); Serial.print(steerSettings.AckermanFix);

    EEPROM.put(10, steerSettings);
    steerSettingsInit();  // Re-Init steer settings
    return;               // no other processing needed
  }  // 0xFC (252) - Steer Settings


  if (packet.data()[3] == 0xFE && packet.length() == 14)        // 0xFE (254) - Steer Data (sent at GPS freq, ie 10hz (100ms))
  {
    //printPgnAnnoucement(packet.data()[3], (char*)"Steer Data", packet.length());

    if (aogGpsToAutoSteerLoopTimerEnabled)
    {
      aogGpsToAutoSteerLoopTimerEnabled = false;
      Serial.print((String)"\r\nGPS out to Steer Data in delay: " + aogGpsToAutoSteerLoopTimer);
    }
    /*Serial.printf(" %6i", micros() - pgn254Time);
    pgn254Time = micros();
    uint32_t pgn254Delay = pgn254Time - nmeaPgnSendTime;
    if (pgn254Delay < pgn254MinDelay) pgn254MinDelay = pgn254Delay;
    if (pgn254Delay > pgn254MaxDelay) pgn254MaxDelay = pgn254Delay;
    if (pgn254AveDelay == 0) pgn254AveDelay = pgn254Delay;
    else pgn254AveDelay = pgn254AveDelay * 0.99 + pgn254Delay * 0.01;
    Serial.printf("->PGN254 delay: %4iuS  %4i %4i %4i", pgn254Delay, pgn254MinDelay, pgn254AveDelay, pgn254MaxDelay);*/
    gpsSpeed = ((float)(packet.data()[5] | packet.data()[6] << 8)) * 0.1;   // speed data comes in as km/hr x10
    //Serial << "\r\n speed:" << gpsSpeed << " "; Serial.print(packet.data()[5], BIN); Serial << " "; Serial.print(packet.data()[6], BIN);
    speedPulse.updateSpeed(gpsSpeed);

    prevGuidanceStatus = guidanceStatus;
    guidanceStatus = packet.data()[7];
    guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

    //Bit 8,9    set point steer angle * 100 is sent
    steerAngleSetPoint = ((float)(packet.data()[8] | ((int8_t)packet.data()[9]) << 8)) * 0.01;  //high low bytes

    //packet.data()[10] is XTE (cross track error)
    //packet.data()[11 & 12] is section 1-16

    if ((bitRead(guidanceStatus, 0) == 0) || (steerState == 0)) { // || (gpsSpeed < 0.1)) {
      watchdogTimer = WATCHDOG_FORCE_VALUE;  //turn off steering motor
      //Serial.print(" OFF");
    } else {                                 //valid conditions to turn on autosteer
      watchdogTimer = 0;                     //reset watchdog
      //Serial.print(" ON");
    }

    //Bit 10 XTE
    xte = packet.data()[10];
    //Serial.print("\r\nXTE:"); Serial.print(xte-127);

    //Bit 11
    //relay = packet.data()[11];

    //Bit 12
    //relayHi = packet.data()[12];

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
    PGN_253[12] = (uint8_t)abs(pwmDisplay);

    //checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < sizeof(PGN_253) - 1; i++)
      CK_A = (CK_A + PGN_253[i]);

    PGN_253[sizeof(PGN_253) - 1] = CK_A;

    //off to AOG
    UDP_Susage.timeIn();
    SendUdpByte(PGN_253, sizeof(PGN_253), broadcastIP, portAgIO_9999);
    UDP_Susage.timeOut();

    //Steer Data 2 -------------------------------------------------
    /*if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
      if (aog2Count++ > 2) {                                // send 1/3 of Steer Data rate (GPS hz / 3)
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
        SendUdpByte(PGN_250, sizeof(PGN_250), broadcastIP, portAgIO_9999);
        UDP_Susage.timeOut();
        aog2Count = 0;
      }
    }*/

    if (aog2Count++ > 1) {                                // send 1/2 of Steer Data rate (GPS hz / 2)
      // fromAutoSteerData FD 250 - sensor values etc
      uint8_t PGN_250[] = { 0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

      if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
        PGN_250[5] = (byte)sensorReading;
      } else {
        PGN_250[5] = (byte)pulseCount;
      }

      CK_A = 0;
      for (uint8_t i = 2; i < sizeof(PGN_250) - 1; i++) {
        CK_A = (CK_A + PGN_250[i]);
      }
      PGN_250[sizeof(PGN_250) - 1] = CK_A;

      UDP_Susage.timeIn();
      SendUdpByte(PGN_250, sizeof(PGN_250), broadcastIP, portAgIO_9999);
      UDP_Susage.timeOut();
      aog2Count = 0;
    }
    return;                     // no other processing needed
  }  // 0xFE (254) - Steer Data



  #ifdef MACHINE_H
    PGNusage.timeOut();
    MACHusage.timeIn();
    //IPAddress ipDest = broadcastIP;
    //uint8_t machineReplyData[] = { 0x80, 0x81, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 23 };  // long enough for machine scan reply len 13, AgIO Hello reply len 11
    //uint8_t machineReplyData[] = { 0x80, 0x81, 123, 203, 7, myIP[0], myIP[1], myIP[2], myIP[3], rem_ip[0], rem_ip[1], rem_ip[2], 23 };
    //uint8_t machineReplyLen = 0;    // set default len of 0, which means there's no reply data to send

    if (machine.parsePGN(packet.data(), packet.length())) //, machineReplyData, &machineReplyLen))//, &ipDest))    // look for Machine PGNs
    {
      pgnMatched = true;
    }
    MACHusage.timeOut();
    PGNusage.timeIn();
  #endif



  if (!pgnMatched) printPgnAnnoucement(packet.data()[3], (char*)"Unprocessed PGN", packet.length());
}



  void nTrip(AsyncUDPPacket packet)
  {
    if (packet.remotePort() != 9999 || packet.length() < 5) return;  //make sure from AgIO
    uint16_t size = packet.length();
    uint8_t NTRIPData[size - 4];
    for (int i = 4; i < size; i++) NTRIPData[i - 4] = packet.data()[i];
    SerialGPS->write(NTRIPData, size - 4);
  }

  void gNSS(AsyncUDPPacket packet)
  {
    if (packet.remotePort() != 9999 || packet.length() < 5) return;  //make sure from AgIO
    Serial.println("Got udpGPS packet");
    // uint16_t size = packet.length();
    // uint8_t NTRIPData[size - 4];
    // for (int i = 4; i < size; i++) nmeaParser << packet.data()[i];

  }

void steerSettingsInit() {
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void steerConfigInit() {
  if (steerConfig.CytronDriver) {
    pinMode(SLEEP_PIN, OUTPUT);
    if (steerConfig.SteerButton == 0 && steerConfig.SteerSwitch == 0) {
      //currentState = 0;
      prevSteerReading = 1;
    }
  }
  
  if (steerConfig.PressureSensor) {
    pinMode(KICKOUT_A_PIN, INPUT_DISABLE);
  } else {
    pinMode(KICKOUT_A_PIN, INPUT_PULLUP);
  }

  BNO.isSwapXY = !steerConfig.IsUseY_Axis;
}

void printPgnAnnoucement(uint8_t _pgnNum, char* _pgnName, uint8_t _len)
{
  Serial.print("\r\n\n0x"); Serial.print(_pgnNum, HEX);
  Serial.print(" ("); Serial.print(_pgnNum); Serial.print(") - ");
  Serial.print(_pgnName); Serial.print(", "); Serial.print(_len); Serial.print(" bytes ");
  Serial.print(millis());

}

  void SendUdpByte(uint8_t* _data, uint8_t _length, IPAddress _ip, uint16_t _port) {
    //PGN.beginPacket(_ip, _port);
    //PGN.write(_data, _length, );
    //PGN.endPacket();
    PGN.writeTo(_data, _length, _ip, _port);
  }

  void SendUdpChar(char* _charBuf, uint8_t _length, IPAddress _ip, uint16_t _port) {
    // PGN.beginPacket(_ip, _port);
    // PGN.write(_charBuf, _length);
    // PGN.endPacket();
    uint8_t tmpBuf[_length];
    uint8_t* tmpBufptr = tmpBuf;
    for (int i = 0; i < _length; i++)
    {      
      tmpBuf[i] = _charBuf[i];
    }
    PGN.writeTo(tmpBufptr, _length, _ip, _port);
  }

   void SendUdpAry(char _data[], uint8_t _length, IPAddress _ip, uint16_t _port) {
    // PGN.beginPacket(_ip, _port);
    // PGN.write(_data, _length);
    // PGN.endPacket();
    uint8_t tmpBuf[_length];
    uint8_t* tmpBufptr = tmpBuf;
    for (int i = 0; i < _length; i++)
    {      
      tmpBuf[i] = _data[i];
    }
    PGN.writeTo(tmpBufptr, _length, _ip, _port);
  } 

  // "raw" method, bypasses limit checks in firmware but AOG should still have limits
  //uint8_t PGN_99[] = { 0x80, 0x81, 126, 0x99, n/u, 1-2, 1-10s, 'H', 'e', 'l', 'l', 'o', ' ', 'A', 'o', 'G', '!', '!' }; //, 0xCC };
  //SendUdpByte(PGN_99, sizeof(PGN_99), broadcastIP, portAgIO_9999);

  // "proper" function
  //char msg[] = "AutoSteer Switch ON";
  //char msgTime = 2;
  //SendUdpFreeForm(1, msg, strlen(msg), msgTime, broadcastIP, portAgIO_9999);  // timed popup

  //char msg[] = "Work switch";
  //SendUdpFreeForm(2, msg, strlen(msg), 1, broadcastIP, portAgIO_9999);        // interactive "OK" popup

  void SendUdpFreeForm(uint8_t _type, char _msg[], uint8_t _len, char _seconds, IPAddress dip, uint16_t dport)
  {
    char header[7] = { 0x80, 0x81, 126, 0x99, 0, 1, 1};   // free form msg PGN header, 126 is steer module ID (not used yet)
    header[5] = _type;

    _seconds = max(1, _seconds);      // limit 1-10 sec msg box time
    _seconds = min(10, _seconds);
    header[6] = _seconds;

    uint8_t ForTheWire[_len + 8];
    //Serial.print("\r\n");
    for (byte i = 0; i < 7; i++) {
      ForTheWire[i] = header[i];
      //Serial.print(ForTheWire[i], DEC); Serial.print(" ");
    }

    for (byte i = 0; i < _len; i++) {
      ForTheWire[i + 7] = _msg[i];
      //Serial.print(ForTheWire[i+7]);
    }

    ForTheWire[_len + 7] = 0;

    if (_type == 1) { Serial.print("\r\n"); Serial.print(ForTheWire[6], DEC); Serial.print("s timed "); }
    else Serial.print("\r\nOK ");

    Serial.print(" pop-up msg sent to AOG: "); Serial.print("\"");
    for (byte i = 7; i < sizeof(ForTheWire) - 1; i++) {
      Serial.print(ForTheWire[i]);
    }
    Serial.print("\"");

    // PGN.beginPacket(dip, dport);
    // PGN.write(ForTheWire, sizeof(ForTheWire)); // +1 to include null terminator
    // PGN.endPacket();
    PGN.writeTo(ForTheWire, sizeof(ForTheWire), dip, dport);
  }

  void SaveModuleIP(void) {
    //ID stored in 60
    EEPROM.put(62, myIP[0]);
    EEPROM.put(63, myIP[1]);
    EEPROM.put(64, myIP[2]);
  }

  void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
  }

};
//#endif
