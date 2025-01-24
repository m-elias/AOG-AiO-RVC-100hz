#include <stdint.h>
/*AiO v5 has machine outputs
    - v5.0a-c uses PCA9555 on Wire1
    - v5.0d uses PCA9685 on Wire
  AiO v4.x fails outputs.begin(I2C_WIRE) so it does not init the machine class but supresses PGN debug msgs

*/

#include <Streaming.h>

const uint8_t drvCnt = 5;
const uint8_t drvSleepPinAssignments[drvCnt] = {
  14, // AUX nSLEEP
  15, // LOCK nSLEEP
  13, // Section 1/2 nSLEEP
   3, // Section 3/4 nSLEEP
   7  // Section 5/6 nSLEEP
};

// set/leave these PCA pins LOW(default) to allow DRV operation or HIGH to for Standby (Hi-z outputs)
const uint8_t drvOffPinAssignments[3] = {
  2,  // Sec 1/2 DRVOFF
  6,  // Sec 3/4 DRVOFF
  8   // Sec 5/6 DRVOFF
};

void outputsInit() {
  Serial << "\r\nInitializing Outputs";

  #ifdef AIOv50d
    Serial << "\r\n- v5.0d PCA9685 I2C PWM IO extender 0x44";// Serial.print(addr, HEX);
    outputs.begin();          // Adafruit_PWMServoDriver 
    Wire.setClock(1000000);
    outputs.setPWMFreq(1526); // the maximum, to hopefully mitigate switching frequency noise
    outputs.setOutputMode(false); // false: open drain, true: totempole (push/pull)

    Wire.beginTransmission(0x44);
    Serial.print("\r\n  - Section DRV8243 ");
    if (Wire.endTransmission() == 0)
      Serial.print("found");
    else
      Serial.print("*NOT found!*");

    Wire.beginTransmission(0x70);
    Serial.print("\r\n  - RGB DRV8243 ");
    if (Wire.endTransmission() == 0)
      Serial.print("found");
    else
      Serial.print("*NOT found!*");

    outputs.setPin(15, 0, 0); // sets PCA9685 pin LOW 0V, DRV Deep Sleep
    outputs.setPin(14, 0, 0); // sets PCA9685 pin LOW 0V, DRV Deep Sleep


  #else
    #ifdef AIOv50a
      Serial << "\r\n- v5.0d PCA9555 I2C IO extender";
    #else
      Serial << "\r\n- v4.x no Section/Machine outputs supported";
    #endif
    if (outputs.begin(I2C_WIRE)) {            // clsPCA9555.cpp
      Serial.print("\r\nSection outputs (PCA9555) detected (8 channels, low side switching)");
    }
  #endif
}

/*
PCA9685 sleep/wake notes

tSLEEP 40-120uS
tWAKEUP 10uS
tCOM 400us max
tRESET 5-20 uS
tREADY 1ms

reset pulse 20-40uS (low pulse)
sleep pulse 120uS-no limit (low pulse)

(deep) Sleep (low power) state occurs when nSLEEP pin is asserted low for > tSLEEP 120uS
  or VDD is low enough for PoR (power on reset) conditions
  drivers are Hi-Z, nFAULT output is de-asserted

Standby state initiates when nSLEEP pin is asserted high for > tWAKEUP (10uS)
  or VDD rises such that internal PoR si released to indicate a power-up
  after tCOM (400uS), it's ready for communication, nFAULT pin is asserted low (lights up the Fault LED)
  after tREADY (1ms), wake-up is complete
  once it receives nSLEEP reset pulse, nFAULT pin is de-asserted and hence forth driver output is based on the bridge mode's truth tables

At power up, with nSLEEP pin low (PCA9685 default), DRV8243 is in Deep Sleep
When nSLEEP is high at power up or goes high while in Deep Sleep, fault LED lights, waiting for nSLEEP reset pulse

*/

void outputsStart(){
  #ifdef AIOv50d
    // if PCA9685, enable AUX output with a 20-40 uS nSLEEP reset pulse at least 1ms after power up
    //while (millis() < 1100){
        //delay(1);
    //}
    for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
      outputs.setPin(drvSleepPinAssignments[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V
    }

    delay(1);

    for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
      outputs.setPin(drvSleepPinAssignments[drvNum], 187, 1); // LOW pulse, 187/4096 is 30uS at 1532hz
    }
    //delayMicroseconds(100);
    uint32_t t1 = micros();
    for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
      outputs.setPin(drvSleepPinAssignments[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V
    }
    uint32_t t2 = micros();
    Serial << "\r\n" << t2 - t1 << "uS";  // 131uS, 4 "resets" issued at a time (131/30 = 4.37)
    Serial << "\r\n4-" << millis();
    //delay(3);

    //machine.init(&outputs, pcaOutputPinNumbers, pcaInputPinNumbers, 100); // mach.h
  #else
    //machine.init(&outputs, pcaOutputPinNumbers, pcaInputPinNumbers, 100); // mach.h
  #endif

}