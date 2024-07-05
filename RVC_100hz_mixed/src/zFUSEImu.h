class FUSE_Imu
{

#define twoPI 6.28318530717958647692
#define PIBy2 1.57079632679489661923

public:
    struct FUSE_Data
    {
        // Fusing BNO with Dual
        double rollDelta;
        double rollDeltaSmooth;
        double correctionHeading;
        double gyroDelta;
        double imuGPS_Offset;
        double gpsHeading;
        double imuCorrected;
        double rollImu = 0;
        double baseline = 0;
        double rollDual = 0;
        double pitchDual = 0;
        double heading = 0;
        bool useFUSEImu = false;
    };
    FUSE_Data fuseData;

    void imuDualDelta()
    {
        // This routine is from original BrianT AOG firmmware
        fuseData.correctionHeading = fuseData.correctionHeading * 0.1 * -DEG_TO_RAD; // correctionHeading is IMU heading in radians
        fuseData.gpsHeading = fuseData.heading * DEG_TO_RAD;                         // gpsHeading is Dual heading in radians
        fuseData.rollImu = fuseData.rollImu * 0.1;                                   // scale rollImu input.

        // Difference between the IMU heading and the GPS heading
        fuseData.gyroDelta = (fuseData.correctionHeading + fuseData.imuGPS_Offset) - fuseData.gpsHeading;
        if (fuseData.gyroDelta < 0)
            fuseData.gyroDelta += twoPI;

        // calculate delta based on circular data problem 0 to 360 to 0, clamp to +- 2 Pi
        if (fuseData.gyroDelta >= -PIBy2 && fuseData.gyroDelta <= PIBy2)
            fuseData.gyroDelta *= -1.0;
        else
        {
            if (fuseData.gyroDelta > PIBy2)
            {
                fuseData.gyroDelta = twoPI - fuseData.gyroDelta;
            }
            else
            {
                fuseData.gyroDelta = (twoPI + fuseData.gyroDelta) * -1.0;
            }
        }
        if (fuseData.gyroDelta > twoPI)
            fuseData.gyroDelta -= twoPI;
        if (fuseData.gyroDelta < -twoPI)
            fuseData.gyroDelta += twoPI;

        // if the gyro and last corrected fix is < 10 degrees, super low pass for gps
        if (abs(fuseData.gyroDelta) < 0.18)
        {
            // a bit of delta and add to correction to current gyro
            fuseData.imuGPS_Offset += (fuseData.gyroDelta * (0.1));
            if (fuseData.imuGPS_Offset > twoPI)
                fuseData.imuGPS_Offset -= twoPI;
            if (fuseData.imuGPS_Offset < -twoPI)
                fuseData.imuGPS_Offset += twoPI;
        }
        else
        {
            // a bit of delta and add to correction to current gyro
            fuseData.imuGPS_Offset += (fuseData.gyroDelta * (0.2));
            if (fuseData.imuGPS_Offset > twoPI)
                fuseData.imuGPS_Offset -= twoPI;
            if (fuseData.imuGPS_Offset < -twoPI)
                fuseData.imuGPS_Offset += twoPI;
        }

        // So here how we have the difference between the IMU heading and the Dual GPS heading
        // This "imuGPS_Offset" will be used in imuHandler() when the GGA arrives

        // Calculate the diffrence between dual and imu roll
        float imuRoll;
        imuRoll = fuseData.rollImu * 0.1;
        fuseData.rollDelta = fuseData.rollDual - imuRoll;
        fuseData.rollDeltaSmooth = (fuseData.rollDeltaSmooth * 0.7) + (fuseData.rollDelta * 0.3);

        fuseData.imuCorrected = fuseData.correctionHeading + fuseData.imuGPS_Offset;
        if (fuseData.imuCorrected > twoPI)
            fuseData.imuCorrected -= twoPI;
        if (fuseData.imuCorrected < 0)
            fuseData.imuCorrected += twoPI;

        fuseData.imuCorrected = fuseData.imuCorrected * RAD_TO_DEG;

        Serial.println("--GPS Data--");
        Serial.print("Heading: ");
        Serial.println(fuseData.heading);
        Serial.print("RollDual: ");
        Serial.println(fuseData.rollDual);
        Serial.println("--IMU Data--");
        Serial.print("correctionHeading: ");
        Serial.println(fuseData.correctionHeading);
        Serial.print("roll: ");
        Serial.println(fuseData.rollImu);
        Serial.println("--Output Data--");
        Serial.print("imuGPS_Offset: ");
        Serial.println(fuseData.imuGPS_Offset);
        Serial.print("rollDeltaSmooth: ");
        Serial.println(fuseData.rollDeltaSmooth);
        Serial.print("imuCorrected: ");
        Serial.println(fuseData.imuCorrected);
    }
};
