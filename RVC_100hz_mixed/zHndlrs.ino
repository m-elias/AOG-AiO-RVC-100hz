// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char nmea[100];

// GGA
struct GGA_DATA {
  char fixTime[12];
  char latitude[15];
  char latNS[3];
  char longitude[15];
  char lonEW[3];
  char fixQuality[2];
  char numSats[4];
  char HDOP[5];
  char altitude[12];
  char ageDGPS[10];
}; GGA_DATA GGA;

// VTG
struct VTG_DATA {
  char heading[12];
  char speedKnots[10];
}; VTG_DATA VTG;

// IMU
struct IMU_DATA {
  char heading[6];
  char roll[6];
  char pitch[6];
  char yawRate[6];
}; IMU_DATA IMU;

//uint32_t nmeaPgnSendTime, nmeaPgnMaxPeriod, nmeaPgnAvePeriod, nmeaPgnMinPeriod = 99999;
//uint8_t nmeaCount;

// If odd characters showed up
void errorHandler() {
  if (!startup) Serial.print("\r\n*** Unexpected characters in NMEA parser ***");
}

void prepImuPandaData()    // run after GGA update + 40ms (timing for PANDA), for next GGA 
{
  if (BNO.isActive) {
    double angVel;
    itoa(BNO.rvcData.yawX10, IMU.heading, 10);    // format IMU data for Panda Sentence - Heading

    if (BNO.isSwapXY) {
        itoa(BNO.rvcData.pitchX10, IMU.roll, 10);   // the pitch x10
        itoa(BNO.rvcData.rollX10, IMU.pitch, 10);   // the roll x10
    } else {
        itoa(BNO.rvcData.pitchX10, IMU.pitch, 10);  // the pitch x10
        itoa(BNO.rvcData.rollX10, IMU.roll, 10);    // the roll x10
    }

    //Serial.print(BNO.angCounter);
    //Serial.print(", ");
    //Serial.print(BNO.rvcData.angVel);
    //Serial.print(", ");
    
    // YawRate
    if (BNO.angCounter > 0) {
        angVel = ((double)BNO.rvcData.angVel) / (double)BNO.angCounter;
        angVel *= 10.0;
        BNO.angCounter = 0;
        BNO.rvcData.angVel = (int16_t)angVel;
    } else {
        BNO.rvcData.angVel = 0;
    }

    itoa(BNO.rvcData.angVel, IMU.yawRate, 10);
    BNO.rvcData.angVel = 0;
  }
  else    // No BNO in RVC mode or its disconnected, set IMU PANDA components to signal AOG that there's no IMU
  {
    itoa(65535, IMU.heading, 10);
    IMU.roll[0] = 0;
    IMU.pitch[0] = 0;
    IMU.yawRate[0] = 0;
  }
}

void GNS_Handler() // Rec'd GNS
{
    NMEA_Pusage.timeIn();
    
    nmeaParser.getArg(0, GGA.fixTime);      // fix time
    nmeaParser.getArg(1, GGA.latitude);     // latitude
    nmeaParser.getArg(2, GGA.latNS);
    nmeaParser.getArg(3, GGA.longitude);    // longitude
    nmeaParser.getArg(4, GGA.lonEW);
    
    //nmeaParser.getArg(5, GGA.fixQuality);   // fix quality
    char temp[4];
    nmeaParser.getArg(5, temp);
    Serial.print("\r\nGNS fix qual: "); Serial.print(temp);
    switch (temp[0]) {
      case 'A':
        itoa(1, GGA.fixQuality, 10);  // 1: autonomous, no correction
        break;
      case 'D':
        itoa(2, GGA.fixQuality, 10);  // 2: differential (WAAS)
        break;
      case 'F':
        itoa(5, GGA.fixQuality, 10);  // 5: FLOAT
        break;
      case 'R':
        itoa(4, GGA.fixQuality, 10);  // 4: RTK FIX
        break;
      case 'E':
        itoa(6, GGA.fixQuality, 10);  // 6: Dead reckoning
        break;
      case 'S':
        itoa(4, GGA.fixQuality, 10);  // ?: Simulator
        break;
      case 'N': default:
        itoa(0, GGA.fixQuality, 10);  // 0: fix not valid
        break;
    }

    nmeaParser.getArg(6, GGA.numSats);      // satellite #
    nmeaParser.getArg(7, GGA.HDOP);         // HDOP
    nmeaParser.getArg(8, GGA.altitude);     // altitude
    nmeaParser.getArg(10, GGA.ageDGPS);     // time of last DGPS update

    Serial.print((String)"\r\n" + millis() + " GNS update ");
    Serial.print(GGA.fixTime);
    triggerGGAGNSFlags();
    gpsLostTimer = 0;                       // Used for GGA timeout (LED's ETC) 
}

void triggerGGAGNSFlags()   // *** needs better name ***
{
  ggaReady = true;                        // we have new GGA sentence
  imuPandaSyncTimer = 0;                  // reset imu timer
  imuPandaSyncTrigger = true;
  startup = true;
  gps1Stats.incHzCount();
  LEDS.setGpsLED(atoi(GGA.fixQuality));   

  if (!ubxParser.useDual) {               // if not using Dual 
    buildPandaOrPaogi(PANDA);             // build the PANDA sentence right away
  }                                       // otherwise wait until relposned arrives in main loop()
}

void GGA_Handler() // Rec'd GGA
{
    NMEA_Pusage.timeIn();
    
    nmeaParser.getArg(0, GGA.fixTime);      // fix time
    nmeaParser.getArg(1, GGA.latitude);     // latitude
    nmeaParser.getArg(2, GGA.latNS);
    nmeaParser.getArg(3, GGA.longitude);    // longitude
    nmeaParser.getArg(4, GGA.lonEW);
    nmeaParser.getArg(5, GGA.fixQuality);   // fix quality
    nmeaParser.getArg(6, GGA.numSats);      // satellite #
    nmeaParser.getArg(7, GGA.HDOP);         // HDOP
    nmeaParser.getArg(8, GGA.altitude);     // altitude
    nmeaParser.getArg(12, GGA.ageDGPS);     // time of last DGPS update

    Serial.print((String)"\r\n" + millis() + " GGA update ");
    Serial.print(GGA.fixTime);
    triggerGGAGNSFlags();
    gpsLostTimer = 0;                       // Used for GGA timeout (LED's ETC) 
}

void VTG_Handler()
{
  nmeaParser.getArg(0, VTG.heading);        // vtg heading
  nmeaParser.getArg(4, VTG.speedKnots);     // vtg Speed knots
}

void buildPandaOrPaogi(bool _panda)    // only called by GGA_Handler (above)
{
    //strcpy(nmea, "");

    if (_panda) strcpy(nmea, "$PANDA,");
    else strcpy(nmea, "$PAOGI,");

    strcat(nmea, GGA.fixTime); strcat(nmea, ",");     // field 1
    strcat(nmea, GGA.latitude); strcat(nmea, ",");
    strcat(nmea, GGA.latNS); strcat(nmea, ",");
    strcat(nmea, GGA.longitude); strcat(nmea, ",");
    strcat(nmea, GGA.lonEW); strcat(nmea, ",");       // 5
    strcat(nmea, GGA.fixQuality); strcat(nmea, ",");

    if (ubxParser.pvtRead) {
      char temp[3];
      itoa(ubxParser.ubxData.numSats, temp, 10);    // ubx pvt has more accurate # SVs, only send it at about 1hz
      strcat(nmea, temp);
    } else {
      strcat(nmea, GGA.numSats);
    }

    strcat(nmea, ",");
    strcat(nmea, GGA.HDOP); strcat(nmea, ",");
    strcat(nmea, GGA.altitude); strcat(nmea, ",");    // 9
    strcat(nmea, GGA.ageDGPS); strcat(nmea, ",");
    strcat(nmea, VTG.speedKnots); strcat(nmea, ",");

    if (_panda) {   // use BNO values
      strcat(nmea, IMU.heading); strcat(nmea, ",");
      strcat(nmea, IMU.roll); strcat(nmea, ",");      // 13
      strcat(nmea, IMU.pitch); strcat(nmea, ",");
      strcat(nmea, IMU.yawRate);
    }
    else {          // use Dual values
      // replace these with Dual baseline calcs
      char temp[6];
      itoa(ubxParser.ubxData.baseRelH, temp, 10);
      strcat(nmea, temp); strcat(nmea, ",");          // 12

      itoa(ubxParser.ubxData.baseRelRoll, temp, 10);
      strcat(nmea, temp); strcat(nmea, ",");          // 13

      strcat(nmea, ""); strcat(nmea, ",");            // blank pitch
      strcat(nmea, "");                               // blank yaw rate
    }

    strcat(nmea, "*");
    CalculateChecksum();
    strcat(nmea, "\r\n");
    NMEA_Pusage.timeOut();

    Serial.print("\r\n"); Serial.print(millis()); Serial.print(" ");// Serial.print(GGA.fixTime); Serial.print(" ");
    Serial.write(nmea);

    if (UDP.isRunning)        //If ethernet running send the GPS there
    {
      //send char stream
      UDP_Susage.timeIn();
      UDP.SendUdpChar(nmea, strlen(nmea), UDP.broadcastIP, UDP.portAgIO_9999);
      UDP_Susage.timeOut();
    }
    else Serial.write(nmea);   // if Eth is !connected, send USB GPS data
}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex2);
}

/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU OR DUAL:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with


  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV

  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
      6A          The checksum data, always begins with

  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
     48          Checksum
*/

