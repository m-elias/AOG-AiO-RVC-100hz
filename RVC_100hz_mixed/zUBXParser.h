#include <stdint.h>
class UBX_Parser {
private:

  typedef enum {
    GOT_NONE,
    GOT_SYNC1,
    GOT_SYNC2,
    GOT_CLASS,
    GOT_ID,
    GOT_LENGTH1,
    GOT_LENGTH2,
    GOT_PAYLOAD,
    GOT_CHKA
  } state_t;

  state_t state;
  int msgclass;
  int msgid;
  int msglen;
  char chka;
  char chkb;
  int count;
  char payload[1000];
  uint32_t prevMsgTime, startMsgTime;

  void addchk(int b) {
    this->chka = (this->chka + b) & 0xFF;
    this->chkb = (this->chkb + this->chka) & 0xFF;
  }


  void dispatchMessage() {

    //Serial.print("\r\nTTRU: "); Serial.println(micros() - startMsgTime);
    msgReadTime = micros() - startMsgTime;

    switch (this->msgid) {
      //POSECEF
      case 0x01:
        {
          unsigned long iTOW = (unsigned long)this->unpack_int32(0);
          long ecefX = this->unpack_int32(4);
          long ecefY = this->unpack_int32(8);
          long ecefZ = this->unpack_int32(12);
          long pAcc = this->unpack_int32(16);
          this->handle_NAV_POSECEF(iTOW, ecefX, ecefY, ecefZ, pAcc);
        }
        break;

      //RELPOSNED
      case 0x3C:
        {
          unsigned long iTOW = (unsigned long)this->unpack_int32(4);
          long relPosN = this->unpack_int32(8);
          long relPosE = this->unpack_int32(12);
          long relPosD = this->unpack_int32(16);
          long relPosLength = this->unpack_int32(20);
          long relPosHeading = this->unpack_int32(24);
          this->handle_NAV_RELPOSNED(iTOW, relPosN, relPosE, relPosD, relPosLength, relPosHeading);
        }
        break;

      //PVT
      case 0x07:
        {
          unsigned long iTOW = (unsigned long)this->unpack_int32(0);
          unsigned short year = (unsigned short)this->unpack_int16(4);
          unsigned char month = (unsigned char)this->payload[6];
          unsigned char day = (unsigned char)this->payload[7];
          unsigned char hour = (unsigned char)this->payload[8];
          unsigned char min = (unsigned char)this->payload[9];
          unsigned char sec = (unsigned char)this->payload[10];
          unsigned char vaild = (unsigned char)this->payload[11];
          unsigned char fixType = (unsigned char)this->payload[20];
          unsigned char flags = (unsigned char)this->payload[21];
          unsigned char flags2 = (unsigned char)this->payload[22];
          unsigned char numSV = (unsigned char)this->payload[23];
          signed long lon = (signed long)this->unpack_int32(24);
          signed long lat = (signed long)this->unpack_int32(28);
          signed long height = (signed long)this->unpack_int32(32);
          signed long hMSL = (signed long)this->unpack_int32(36);
          signed long velN = (signed long)this->unpack_int32(48);
          signed long velE = (signed long)this->unpack_int32(52);
          signed long velD = (signed long)this->unpack_int32(56);
          signed long gSpeed = (signed long)this->unpack_int32(60);
          signed long headMot = (signed long)this->unpack_int32(64);
          signed long headVeh = (signed long)this->unpack_int32(84);

          this->handle_NAV_PVT(iTOW, year, month, day, hour, min, sec, vaild, fixType, flags, flags2, numSV,
                               lon, lat, height, hMSL, velN, velE, velD, gSpeed, headMot, headVeh);
        }
        break;

      default:
        this->reportUnhandled(this->msgid);
        break;
    }
  }

  long unpack_int32(int offset) {
    return this->unpack32(offset, 4);
    ;
  }

  long unpack_int16(int offset) {
    return this->unpack16(offset, 2);
    ;
  }

  long unpack16(int offset, int size) {
    long value = 0;  // four bytes on most Arduinos

    for (int k = 0; k < size; ++k) {
      value <<= 8;
      value |= (0xFF & this->payload[offset + 2 - k - 1]);
    }
    return value;
  }

  long unpack32(int offset, int size) {
    long value = 0;  // four bytes on most Arduinos

    for (int k = 0; k < size; ++k) {
      value <<= 8;
      value |= (0xFF & this->payload[offset + 4 - k - 1]);
    }
    return value;
  }

public:

  struct UBX_Data {

    float ecefX, ecefY, ecefZ, ecefAcc;                      // rover ECEF postion in meters
    float baseRelN, baseRelE, baseRelD, baseRelL, baseRelH;  // moving base relposned in meters
    uint32_t iTOW;                                           // time of week from relposned
    byte numSats;                                             // number of Sats from PVT
    float lat, lon, alt;                                     // position from PVT
  };
  UBX_Data ubxData;

  bool relPosNedReady, useDual, pvtRead;
  uint32_t msgPeriod, msgReadTime;

  void handle_NAV_POSECEF(unsigned long iTOW, long ecefX, long ecefY, long ecefZ, long pAcc) {
    ubxData.ecefX = (float)ecefX * 0.01;
    ubxData.ecefY = (float)ecefY * 0.01;
    ubxData.ecefZ = (float)ecefZ * 0.01;
    ubxData.ecefAcc = (float)pAcc * 0.01;
  }

  void handle_NAV_RELPOSNED(unsigned long iTOW,
                            long relPosN,
                            long relPosE,
                            long relPosD,
                            long relPosLength,
                            long relPosHeading) {
    ubxData.iTOW = iTOW;
    ubxData.baseRelN = (float)relPosN * 0.01;
    ubxData.baseRelE = (float)relPosE * 0.01;
    ubxData.baseRelD = (float)relPosD * 0.01;
    ubxData.baseRelL = (float)relPosLength * 0.01;
    ubxData.baseRelH = (float)relPosHeading * 0.00001;
    //Serial.print(millis()); Serial.print(" handle_NAV_RELPOSNED "); Serial.println((micros() - prevMsgTime) / 1000);
    Serial.print("\r\n"); Serial.print(millis());// Serial.print(" "); Serial.print(iTOW);
    Serial.print(" handle_NAV_RELPOSNED "); Serial.print((micros() - prevMsgTime) / 1000);
    msgPeriod = (micros() - prevMsgTime) / 1000;
    prevMsgTime = micros();
    relPosNedReady = true;
    useDual = true;
  }

  void handle_NAV_PVT(unsigned long iTOW,
                      unsigned short year,
                      unsigned char month,
                      unsigned char day,
                      unsigned char hour,
                      unsigned char min,
                      unsigned char sec,
                      unsigned char vaild,
                      unsigned char fixType,
                      unsigned char flags,
                      unsigned char flags2,
                      unsigned char numSV,
                      signed long lon,
                      signed long lat,
                      signed long height,
                      signed long hMSL,
                      signed long velN,
                      signed long velE,
                      signed long velD,
                      signed long gSpeed,
                      signed long headMot,
                      signed long headVeh) {

    ubxData.numSats = (byte)numSV;
    ubxData.lat = (float)lat * 0.0000001;
    ubxData.lon = (float)lon * 0.0000001;
    ubxData.alt = (float)height * 0.001;
    Serial.print("\r\n"); Serial.print(millis()); Serial.print(" handle_NAV_PVT ");// Serial.print((micros() - prevMsgTime) / 1000);
    //prevMsgTime = micros();
    pvtRead = true;
  }

  void reportUnhandled(char msgid) {
    Serial.print("\r\nGot message ID:");
    Serial.print(msgid & 0xFF, HEX);
    Serial.print(" cls:");
    Serial.print(msgclass & 0xFF, HEX);
  }

  /**
          * Constructs a UBX parser. 
          */
  UBX_Parser() {
    this->state = GOT_NONE;
    this->msgclass = -1;
    this->msgid = -1;
    this->msglen = -1;
    this->chka = -1;
    this->chkb = -1;
    this->count = 0;
  }

  /**
          * Parses a new byte from the GPS. Automatically calls handle_ methods when a new
          * message is successfully parsed.
          * @param b the byte
          */
  void parse(int b) {
    if (b == 0xB5) {

      this->state = GOT_SYNC1;
      startMsgTime = micros();
    }

    else if (b == 0x62 && this->state == GOT_SYNC1) {

      this->state = GOT_SYNC2;
      this->chka = 0;
      this->chkb = 0;
    }

    else if (this->state == GOT_SYNC2) {

      this->state = GOT_CLASS;
      this->msgclass = b;
      this->addchk(b);
    }

    else if (this->state == GOT_CLASS) {

      this->state = GOT_ID;
      this->msgid = b;
      this->addchk(b);
    }

    else if (this->state == GOT_ID) {

      this->state = GOT_LENGTH1;
      this->msglen = b;
      this->addchk(b);
    }

    else if (this->state == GOT_LENGTH1) {

      this->state = GOT_LENGTH2;
      this->msglen += (b << 8);
      this->count = 0;
      this->addchk(b);
    }

    else if (this->state == GOT_LENGTH2) {

      this->addchk(b);
      this->payload[this->count] = b;
      this->count += 1;

      if (this->count == this->msglen) {

        this->state = GOT_PAYLOAD;
      }
    }

    else if (this->state == GOT_PAYLOAD) {

      this->state = (b == this->chka) ? GOT_CHKA : GOT_NONE;
    }

    else if (this->state == GOT_CHKA) {

      if (b == this->chkb) {
        this->dispatchMessage();
      }

      else {
        this->state = GOT_NONE;
      }
    }
  }
};
