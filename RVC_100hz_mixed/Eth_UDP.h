#include <stdint.h>
#include "elapsedMillis.h"
#include "IPAddress.h"
#ifndef _ETHER_h
#define _ETHER_h

#include "Arduino.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <EEPROM.h>

class Eth_UDP
{
public:
	IPAddress myIP = { 192, 168, 5, 126 };  // 126 default IP for steer module
  IPAddress broadcastIP;
  byte mac[6] = { 0x0A, 0x0F, myIP[0], myIP[1], myIP[2], myIP[3] };     // create unique MAC from IP as IP should already be unique


	// This modules listens to GPS sent on (carry over from Ace)
  // likely not needed but may be convenient for simulating a GPS receiver on the bench using UDP
	unsigned int portNMEA_2211 = 2211;     // Why 2211? 22XX=GPS then 2211=GPS1 2222=GPS2 2233=RTCM3 corrections easy to remember.
	EthernetUDP NMEA;                      // UDP object for incoming NMEA

	unsigned int portRTCM_2233 = 2233;     // Why 2211? 22XX=GPS then 2211=GPS1 2222=GPS2 2233=RTCM3 corrections easy to remember.
	EthernetUDP RTCM;                      // UDP object for incoming RTCM
  
	unsigned int portSteer_8888 = 8888;    // UDP port that Modules (like this one) listens to
	EthernetUDP PGN;                       // UDP object for PGNs on port 8888
	unsigned int portAgIO_9999 = 9999;     // UDP port that AgIO listens to, send data here

	bool isRunning = false;                // set true with successful Eth Start()
  int8_t linkStatus = -1;                // 0 - Unknown, 1 - LinkON, 2 - LinkOFF
	const int EE_ver = 2402;               // if value in eeprom does not match, overwrite with defaults

  elapsedMillis initTimer = 2000;

  Eth_UDP(void) {                        //constructor
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
    }

    broadcastIP[0] = myIP[0];
    broadcastIP[1] = myIP[1];
    broadcastIP[2] = myIP[2];
    broadcastIP[3] = 255;                // same subnet as module's IP but use broadcast
  }
  ~Eth_UDP(void) {}                      //destructor

  bool init()
  {
    if (isRunning) return true;
    //Ethernet.MACAddress(mac);                 // get Teensy's internal MAC, doesn't work reliably
    //Ethernet.begin(mac, 2000, 2000);          // start dhcp connection with 2s timeout, that's enough time to get an eth linkStatus update
    //Ethernet.begin(mac, myIP);                // blocks if unplugged
    Ethernet.begin(mac, 0);                     // non-blocking method, set IP later

    // Check for Ethernet hardware present, always returns "EthernetW5500" (3) for Teensy 4.1 w/Eth
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("\r\n\n*** Ethernet was not found. GPS via USB only ***");   // maybe using non Ethernet Teensy?
      return false;
    }

    Ethernet.setLocalIP(myIP);                  // also non-blocking as opposed to Ethernet.begin(mac, myIP) which block with unplugged/unconnected cable
    Serial.print("\r\n\nEthernet connection set with static IP address");

    Serial.print("\r\n- Using MAC address: ");
    for (byte octet = 0; octet < 6; octet++) {
      if (mac[octet] < 0x10) Serial.print("0");
      Serial.print(mac[octet], HEX);
      if (octet < 5) Serial.print(':');
    }
    /*Serial.print("\r\n- Using IP address: ");
    for (byte octet = 0; octet < 4; octet++) {
      Serial.print(myIP[octet]);
      if (octet < 3) Serial.print('.');
    }*/

    Serial.print("\r\n- Ethernet IP of module: ");
    Serial.print(Ethernet.localIP());

    Serial.print("\r\n- Ethernet Broadcast IP: ");
    Serial.print(broadcastIP);

    Serial.print("\r\n\nSetting up UDP comms");
    Serial.print("\r\n- Sending to AgIO port: ");
    Serial.print(portAgIO_9999);

    if (NMEA.begin(portNMEA_2211)) {
      Serial.print("\r\n- Ethernet UDP GPS listening on port: ");
      Serial.print(portNMEA_2211);
    }

    if (RTCM.begin(portRTCM_2233)) {
      Serial.print("\r\n- Ethernet UDP RTCM listening on port: ");
      Serial.print(portRTCM_2233);
    }

    // init UPD Port getting AutoSteer (8888) data from AGIO
    if (PGN.begin(portSteer_8888)) {
      Serial.print("\r\n- Ethernet UDP PGN listening to port: ");
      Serial.print(portSteer_8888);
    }

    isRunning = true;
    return true;
  }

  void SendUdpByte(uint8_t* _data, uint8_t _length, IPAddress _ip, uint16_t _port) {
    PGN.beginPacket(_ip, _port);
    PGN.write(_data, _length);
    PGN.endPacket();
  }

  void SendUdpChar(char* _charBuf, uint8_t _length, IPAddress _ip, uint16_t _port) {
    PGN.beginPacket(_ip, _port);
    PGN.write(_charBuf, _length);
    PGN.endPacket();
  }



  // "raw" method, bypasses limit checks in firmware but AOG should still have limits
  //uint8_t PGN_99[] = { 0x80, 0x81, 126, 0x99, 2, 'H', 'e', 'l', 'l', 'o', ' ', 'A', 'o', 'G', '!', '!' }; //, 0xCC };
  //UDP.SendUdpByte(PGN_99, sizeof(PGN_99), UDP.broadcastIP, UDP.portAgIO_9999);

  // "proper" function
  //char msg[] = "AutoSteer Btn";
  //char msgTime = 2;
  void SendUdpFreeForm(char _msg[], uint8_t _len, char _seconds, IPAddress dip, uint16_t dport)
  {
    char header[5] = { 0x80, 0x81, 126, 0x99};   // free form msg PGN header, 126 is steer module ID (not used yet)
    char ForTheWire[_len + 6];

    _seconds = max(1, _seconds);      // limit 1-10 sec msg box time
    _seconds = min(10, _seconds);
    header[4] = _seconds;

    for (byte i = 0; i < 5; i++) {
      ForTheWire[i] = header[i];
    }

    for (byte i = 0; i < _len; i++) {
      ForTheWire[i + 5] = _msg[i];
    }

    ForTheWire[_len + 5] = 0;

    Serial.print("\r\nPop-up msg sent to AOG: ");
    Serial.print(ForTheWire[4], DEC);
    Serial.print("s \"");
    for (byte i = 5; i < strlen(ForTheWire); i++) {
      Serial.print(ForTheWire[i]);
    }
    Serial.print("\"");

    PGN.beginPacket(dip, dport);
    PGN.write(ForTheWire, strlen(ForTheWire)); // +1 to include null terminator
    PGN.endPacket();

  }

  void SaveModuleIP(void) {
    //ID stored in 60
    EEPROM.put(62, myIP[0]);
    EEPROM.put(63, myIP[1]);
    EEPROM.put(64, myIP[2]);
  }


};
#endif
