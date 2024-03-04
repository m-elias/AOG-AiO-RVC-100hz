// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char nmea[100];

// GGA
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

// VTG
char vtgHeading[12] = { };
char speedKnots[10] = { };

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

uint32_t nmeaPgnSendTime, nmeaPgnMaxPeriod, nmeaPgnAvePeriod, nmeaPgnMinPeriod = 99999;
uint8_t nmeaCount;

// If odd characters showed up.
void errorHandler()
{
  Serial.print("\r\n*** Odd characters in NMEA parser ***");
  //nothing at the moment
}

void GGA_Handler() //Rec'd GGA
{
    NMEA_Pusage.timeIn();
    // fix time
    nmeaParser.getArg(0, fixTime);

    // latitude
    nmeaParser.getArg(1, latitude);
    nmeaParser.getArg(2, latNS);

    // longitude
    nmeaParser.getArg(3, longitude);
    nmeaParser.getArg(4, lonEW);

    // fix quality
    nmeaParser.getArg(5, fixQuality);

    // satellite #
    nmeaParser.getArg(6, numSats);

    // HDOP
    nmeaParser.getArg(7, HDOP);

    // altitude
    nmeaParser.getArg(8, altitude);

    // time of last DGPS update
    nmeaParser.getArg(12, ageDGPS);

    //we have new GGA sentence
    isGGA_Updated = true;

    //reset imu timer
    imuDelayTimer = 0;

    //build the PANDA sentence
    BuildNmea();

    teensyLedToggle();
    LEDTimer = 0;

    //Used for GGA timeout (LED's ETC) 
    gpsLostTimer = 0;
}

void VTG_Handler()
{
  // vtg heading
  nmeaParser.getArg(0, vtgHeading);

  // vtg Speed knots
  nmeaParser.getArg(4, speedKnots);
}

void imuHandler()
{
    double angVel;

    // Fill rest of Panda Sentence - Heading
    itoa(bnoData.yawX10, imuHeading, 10);

    if (RVC_BNO.isSwapXY)
    {
        // the pitch x100
        itoa(bnoData.pitchX10, imuRoll, 10);

        // the roll x100
        itoa(bnoData.rollX10, imuPitch, 10);
    }
    else
    {
        // the pitch x100
        itoa(bnoData.pitchX10, imuPitch, 10);

        // the roll x100
        itoa(bnoData.rollX10, imuRoll, 10);
    }

    //Serial.print(RVC_BNO.angCounter);
    //Serial.print(", ");
    //Serial.print(bnoData.angVel);
    //Serial.print(", ");
    
    // YawRate
    if (RVC_BNO.angCounter > 0)
    {
        angVel = ((double)bnoData.angVel) / (double)RVC_BNO.angCounter;
        angVel *= 10.0;
        RVC_BNO.angCounter = 0;
        bnoData.angVel = (int16_t)angVel;
    }
    else
    {
        bnoData.angVel = 0;
    }

    itoa(bnoData.angVel, imuYawRate, 10);
    bnoData.angVel = 0;
}

void BuildNmea(void)
{
    strcpy(nmea, "");
    
    strcat(nmea, "$PANDA,");

    strcat(nmea, fixTime);
    strcat(nmea, ",");

    strcat(nmea, latitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, longitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");

    // 6
    strcat(nmea, fixQuality);
    strcat(nmea, ",");

    //strcat(nmea, numSats);
    strcat(nmea, "31");
    strcat(nmea, ",");

    strcat(nmea, HDOP);
    strcat(nmea, ",");

    strcat(nmea, altitude);
    strcat(nmea, ",");

    //10
    strcat(nmea, ageDGPS);
    strcat(nmea, ",");

    //11
    strcat(nmea, speedKnots);
    strcat(nmea, ",");

    //12
    strcat(nmea, imuHeading);
    strcat(nmea, ",");

    //13
    strcat(nmea, imuRoll);
    strcat(nmea, ",");

    //14
    strcat(nmea, imuPitch);
    strcat(nmea, ",");

    //15
    strcat(nmea, imuYawRate);

    strcat(nmea, "*");

    CalculateChecksum();

    strcat(nmea, "\r\n");

    NMEA_Pusage.timeOut();

    //If ethernet running send the GPS there
    if (UDP.isRunning)   
    {
        //send char stream
        UDP_Susage.timeIn();
        UDP.SendUdpChar(nmea, strlen(nmea), UDP.broadcastIP, UDP.portAgIO_9999);
        UDP_Susage.timeOut();
        /*nmeaCount++;
        if (nmeaCount == 10) {
          nmeaPgnMinPeriod = 999999;
          nmeaPgnMaxPeriod = 0;
          nmeaPgnAvePeriod = 0;
        }
        //Serial.printf("\nNMEA->%6i", micros() - nmeaPgnSendTime);
        uint32_t nmeaPgnPeriod = micros() - nmeaPgnSendTime;
        nmeaPgnSendTime = micros();
        if (nmeaPgnPeriod < nmeaPgnMinPeriod) nmeaPgnMinPeriod = nmeaPgnPeriod;
        if (nmeaPgnPeriod > nmeaPgnMaxPeriod) nmeaPgnMaxPeriod = nmeaPgnPeriod;
        if (nmeaPgnAvePeriod == 0) nmeaPgnAvePeriod = nmeaPgnPeriod;
        else nmeaPgnAvePeriod = nmeaPgnAvePeriod * 0.99 + nmeaPgnPeriod * 0.01;
        Serial.printf("\nNMEA->period: %6iuS  %6i %6i %6i", nmeaPgnPeriod, nmeaPgnMinPeriod, nmeaPgnAvePeriod, nmeaPgnMaxPeriod);
        */
    }
    else
    {
        //send USB GPS data
        Serial.write(nmea);  
    }
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

  FROM IMU:
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

