#include <stdint.h>
#ifndef JD_DAC_H
#define JD_DAC_H

//#include "Streaming.h"
#include "MCP4728_DAC.h"
#include "zADS1115.h"
#include <Wire.h>
#include "Arduino.h"

MCP4728 dac;
ADS1115_lite dac_ads(ADS1115_ADDRESS_ADDR_VDD); // addr line pulled to VDD (0x49) ADS1115_ADDRESS_ADDR_VDD

class JD_DAC
{
public:
	JD_DAC(TwoWire& _wirePort, uint8_t _dacAddr = 0x64, Stream* _stream = NULL)
	{
		i2cPort = &_wirePort;
		dacAddr = _dacAddr;
		stream = _stream;
		//dacSetCenterIO = _dacSetCenterIO;
		//dacDevBtn = _dacPrintIO;
		//pinMode(dacSetCenterIO, INPUT_PULLUP);
		//pinMode(dacDevBtn, INPUT_PULLUP);
	}

	void update() {

    if (!isInit && initRetryTimer > 2000){
      initRetryTimer = 0;
      Serial.println("\r\n\nAttempting JD DAC init");
      if (init()){
        Serial.println("-JD DAC init successful");
      } else {
        Serial.println("-JD DAC init failed");
      }
    } else {
      //uint32_t time1 = millis();
      //if (dac.testConnection() == 0){   // only takes 0-1ms
        //Serial.print(millis()-time1);
        if (updateAdsReadingCh0()) {
          //if (printSWS) printSWSdata();
          //printSWSdata();
          //Serial.print(" ads update ");
        }
        //ch4Output();
        //Serial.print(" "); Serial.println(millis()-time1);
      /*} else {
        isInit = 0;
        i2cPort->end();
      }*/
    }
  

		/*if (digitalRead(dacSetCenterIO) == LOW) {
			while (digitalRead(dacSetCenterIO) == LOW) delay(5);
			byte i = NUM_SWS;
			debugPrint("\r\n");
			while (i--) {
				*(steeringWheelSensorCenter + i) = *(steeringWheelSensor + i);
				debugPrint(steeringWheelSensorCenter[i]); debugPrint(" ");
			}

      printStatus();
      printSWS = !printSWS;
		}

		if (digitalRead(dacDevBtn) == LOW) {
			while (digitalRead(dacDevBtn) == LOW) delay(5);

    toolSteerEnable(!ch4Enabled);
			
		}*/


	}

	bool init() {
    isInit = false;
		debugPrint("-checking for Adafruit MCP4728 @ addr 0x");
		debugPrint(dacAddr, HEX);
		dac.setAddr(dacAddr);
    dac.attach(*i2cPort);
		i2cPort->begin();
		uint8_t error = dac.testConnection();
		if (error) {
			debugPrint("\r\n--error: ");
			debugPrint(error);
			debugPrint("\r\n");
			i2cPort->end();
			return false;
		}
		debugPrint("\r\n--MCP4728 found!");
		debugPrint("\r\n--MCP pre init\r\n");
		/*printStatus();

		// set live modes, Live modes are loaded from EE right after saving to EE below but only if EE is updated
		dac.selectVref(MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD);
		dac.selectGain(MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1);
		dac.selectPowerDown(DEFAULT_PWR_DOWN, DEFAULT_PWR_DOWN, DEFAULT_PWR_DOWN, MCP4728::PWR_DOWN::NORMAL);
*/
		printStatus();

		// check/set EE modes
		// could use single EE write to all channel instead of multiple to save some time but after the first time, shouldn't need it again
    debugPrint("\r\nSetting DAC defaults\r\n");
		
    // write defaults to steering channels #0-2 and read steering center values from EE
    for (int i = 0; i < 3; ++i) {
      dac.writeSingleChEepromModes(i, MCP4728::VREF::VDD, MCP4728::GAIN::X1, MCP4728::PWR_DOWN::GND_500KOHM);
  		delay(50);  // necessary to wait for EE to finish as not using RDY IO
      steeringWheelSensorCenter[i] = dac.getDACData(i, true);
		}

    // write defaults to SCV/remote control channel #3
    dac.analogWrite(3, 2047, true);
    dac.writeSingleChEepromModes(3, MCP4728::VREF::VDD, MCP4728::GAIN::X1, MCP4728::PWR_DOWN::NORMAL);
    delay(50);  // necessary to wait for EE to finish as not using RDY IO


		printStatus();
		debugPrint("\r\n--MCP post init");

		debugPrint("\r\n-testing for JD_DAC ADS1115 @ addr 0x49 (addr line pulled to VDD)");
		if (dac_ads.testConnection()) {
			debugPrint("\r\n--DAC ADS1115 Connection GOOD");
		} else {
			debugPrint("\r\n--DAC ADS1115 Connection FAILED!\r\n");
			return false;
		}

		dac_ads.setSampleRate(ADS1115_REG_CONFIG_DR_860SPS); // 860 samples per second
		dac_ads.setGain(ADS1115_REG_CONFIG_PGA_6_144V);      // for 6.144V input, JD SWS don't output higher then 4.1 V but DAC can output 5V
		dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);	   // single ended inputs
		dac_ads.triggerConversion(true);
		lastAdsUpdateTime = millis();
		adsIndex = 0;
		debugPrint("\r\n");
		isInit = true;
		return true;
	}

	int16_t steerOutput(int16_t _tractorPWM) {	// pwmDrive +-(minPWM - Max Limit)
		if (steerOutputEnabled) {
      // 0/2047 is center, 254/3554 is one extreme (old values for #0/1)
      // 0/2047 is center, 254/542 is the other extreme (old values for inverted #2)

      // could use better mapping, maybe use SWS values from a cal proceedure?
      uint16_t output0, output1, output2invt;
			output0 = map(_tractorPWM, 0, 254, steeringWheelSensorCenter[0], 3554);
      output1 = map(_tractorPWM, 0, 254, steeringWheelSensorCenter[1], 3554);
			output2invt = map(_tractorPWM, 0, 254, steeringWheelSensorCenter[2], 542);

			dac.analogWrite(output0, output1, output2invt);	// set all 3 channels, then do single i2c write loop

      debugPrint("\r\n_tractorPWM: ");
			debugPrint(_tractorPWM);
			debugPrint(" -> ");
			debugPrint(output0);
			debugPrint(":");
			debugPrint(output1);
			debugPrint(":");
			debugPrint(output2invt);

      //return output0 << 4;
    }

   return _tractorPWM;
  }

	void ch4Output(int16_t _toolPWM) {
		if (ch4Enabled) {
      // 0.5v - 2.25v and 2.75v - 4.5v
      //  430 - 1932  and  2360 - 3863
      uint16_t ch4Output = map(_toolPWM, 0, 254, 2047, 3863);
      ch4Output = min(ch4Output, 3863);   // limit to max of 3863/4.5v
      ch4Output = max(ch4Output, 430);    // limit to min of 430/0.5v
      dac.analogWrite(3, ch4Output, MCP4728::PWR_DOWN::NORMAL);
			/*if (millis() > sweepTriggerTime) {
				sweep += 1;
				if (sweep > 4090) sweep = 0;
				dac.analogWrite(3, sweep, MCP4728::PWR_DOWN::NORMAL);
				sweepTriggerTime += 20;
				if (millis() > sweepTriggerTime) sweepTriggerTime = millis() + 50;
			}*/
		}
	}

	void steerEnable(bool _enable) {
    if (isInit) {
  		if (_enable != steerOutputEnabled) {
  			if (_enable) {
  				debugPrint("\r\nSteer DAC output enabled");
  			} else {
          // set steering outputs to hiZ output, tool steer output left as is
  				dac.selectPowerDown(DEFAULT_PWR_DOWN, DEFAULT_PWR_DOWN, DEFAULT_PWR_DOWN, MCP4728::PWR_DOWN(dac.getPowerDown(3)));
  				debugPrint("\r\nSteer DAC output disabled");
  			}
  			steerOutputEnabled = _enable;
  		}
		}
	}

  void ch4Enable(bool _enable) {
    if (isInit){
      if (_enable != ch4Enabled) {
        if (_enable) {
          debugPrint("\r\nDAC Ch4 tool steer output enabled");
        } else {
          dac.analogWrite(3, 2047, MCP4728::PWR_DOWN::NORMAL);  // center position for JD remote SCV signal
          debugPrint("\r\nDAC Ch4 tool steer output disabled");
        }
        ch4Enabled = _enable;
      }
    }
  }

  int16_t getWAS(){
    if (isInit) return steeringWheelSensor[0]; // stored ch 0 reading
    else return 13600; // center value
  }

  void readAllSWS(){
    uint16_t readings[3];
    uint16_t readingsConv[3];

    // SWS #0 is already continuously read, so only need to read #1-2
    dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);
    dac_ads.triggerConversion(false);
    while (!dac_ads.isConversionDone());
    readings[1] = dac_ads.getConversion();
    readingsConv[1] = (int)((float)readings[1] / adsToDacConvFactor);

    dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);
    dac_ads.triggerConversion(false);
    while (!dac_ads.isConversionDone());
    readings[2] = dac_ads.getConversion();
    readingsConv[2] = (int)((float)readings[2] / adsToDacConvFactor);

    dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  // set ADS back to reading #0 continuously
    dac_ads.triggerConversion(true);
    readings[0] = steeringWheelSensor[0];
    readingsConv[0] = (int)((float)readings[0] / adsToDacConvFactor);

    Serial.print("\r\n"); Serial.print(millis());
    for (byte i = 0; i < 3; i++){
      Serial.printf(" %i:%i/%i ", i, readings[i], readingsConv[i]);
    }
  }


  void centerDac(){
    Serial.print("\r\n"); Serial.print(millis());
    for (byte i = 0; i < NUM_SWS - 1; i++){
      Serial.printf(" %i:%i ", i, steeringWheelSensorCenter[i]);
    }

    // SWS #0 is already continuously read, so only need to read #1-2
    dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);
    dac_ads.triggerConversion(false);
    while (!dac_ads.isConversionDone());
    steeringWheelSensorCenter[1] = (int)((float)dac_ads.getConversion() / adsToDacConvFactor);
    dac.analogWrite(1, steeringWheelSensorCenter[1], true);

    dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);
    dac_ads.triggerConversion(false);
    while (!dac_ads.isConversionDone());
    steeringWheelSensorCenter[2] = (int)((float)dac_ads.getConversion() / adsToDacConvFactor);
    dac.analogWrite(2, steeringWheelSensorCenter[2], true);

    dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  // set ADS back to reading #0 continuously
    dac_ads.triggerConversion(true);
    steeringWheelSensorCenter[0] = (int)((float)steeringWheelSensor[0] / adsToDacConvFactor); // copy previously saved #0 value
    dac.analogWrite(0, steeringWheelSensorCenter[0], true);

    Serial.print("\r\n"); Serial.print(millis());
    for (byte i = 0; i < NUM_SWS - 1; i++){
      Serial.printf(" %i:%i ", i, steeringWheelSensorCenter[i]);
    }

  }
/*
	bool updateAdsReadings4chSeq(){
		if (dac_ads.isConversionDone()) {	// isConvDone only takes a split uS to return false so hammer away
			currentAdsUpdateTime = millis();
			debugPrint("\r\n"); debugPrint(currentAdsUpdateTime); debugPrint(" "); debugPrint(currentAdsUpdateTime - lastAdsUpdateTime);
			lastAdsUpdateTime = currentAdsUpdateTime;

			steeringWheelSensor[adsIndex] = dac_ads.getConversion();// +adsOffset[adsIndex];
			if (steeringWheelSensor[adsIndex] > 60000) {
        Serial.print("\r\n*** "); Serial.print(adsIndex); Serial.print(" too high "); Serial.print(steeringWheelSensor[adsIndex]);
        steeringWheelSensor[adsIndex] = 0;
      }

			if (adsIndex == NUM_SWS - 1) {
				dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
			}
			else if (adsIndex == 0) {
				dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);
			}
			else if (adsIndex == 1) {
				dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);
			}
			else if (adsIndex == 2) {
				dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3);
			}
			dac_ads.triggerConversion(false);

			adsIndex++;
			if (adsIndex == NUM_SWS) {
				//printSWSdata();
				adsIndex = 0;
				return true;
			}
		}

		return false;
	}*/

  bool updateAdsReadingCh0(){
    // 32-33 ms per channel @ 32SPS, 96-97 ms to update 3 analog channels
    // 9-10 ms per channel @ 128SPS, 36-37 ms to update 4 analog channels
    // 1.2 ms per channel @ 860SPS
    if (adsIndex == 0){
      //Serial.print("\r\nadsIndex = 0");
      //if (dac_ads.isConversionDone()) { // isConvDone only takes a 0-1ms to return false so hammer away
        //Serial.print("\r\nconversionDone");
        //currentAdsUpdateTime = micros();
        //debugPrint("\r\n"); debugPrint(currentAdsUpdateTime); debugPrint(" "); debugPrint(currentAdsUpdateTime - lastAdsUpdateTime);
        //lastAdsUpdateTime = currentAdsUpdateTime;
  
        steeringWheelSensor[0] = dac_ads.getConversion();// +adsOffset[adsIndex];
        if (steeringWheelSensor[0] > 60000) steeringWheelSensor[0] = 0;
  
        dac_ads.triggerConversion(true);
        return true;
      /*} else {
        Serial.print("\r\nconversion !done");
        Serial.println(dac_ads.getConversion());
      }*/
    } else {
      //Serial.print("\r\nadsIndex = "); Serial.print(adsIndex); Serial.print(", changing to 0");
      dac_ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
      dac_ads.triggerConversion(true);
      adsIndex = 0;
    }
    return false;
  }


	void printSWSdata() {
		currentPrintTime = millis();
		//debugPrint("\r\n-ads update cycle complete");
		if (currentPrintTime > lastPrintTime + 1000)
		{
			debugPrint("\r\n"); debugPrint(currentPrintTime); debugPrint("ms ("); debugPrint(currentPrintTime - lastPrintTime); debugPrint(") ");
			for (byte n = 0; n < NUM_SWS; n++) {
				debugPrint(" ");
				/*debugPrint(n); debugPrint(": ");*/
				if (steeringWheelSensor[n] < 10000) debugPrint(" ");
				if (steeringWheelSensor[n] < 1000) debugPrint(" ");
				if (steeringWheelSensor[n] < 100) debugPrint(" ");
				if (steeringWheelSensor[n] < 10) debugPrint(" ");
				debugPrint(steeringWheelSensor[n]);
			}
			lastPrintTime += 1000;
			if (currentPrintTime > lastPrintTime) lastPrintTime = currentPrintTime + 1000;
		}
		//lastPrintTime = currentPrintTime;
	}

	void setDebugStream(Stream* _stream) {
		stream = _stream;
	}

	void printStatus() {
		delay(50);
		dac.readRegisters();
		delay(50);

		debugPrint("\r\nDAC#\tVref\tGain\tPD\tDACData\r\n");
		for (int i = 0; i < 4; ++i) {
			debugPrint(i, DEC);
			debugPrint("\t");

			debugPrint(dac.getVref(i), DEC);
			debugPrint("[");
			debugPrint(dac.getVref(i, true), DEC);
			debugPrint("]\t");

			debugPrint(dac.getGain(i), DEC);
			debugPrint("[");
			debugPrint(dac.getGain(i, true), DEC);
			debugPrint("]\t");

			debugPrint(dac.getPowerDown(i), DEC);
			debugPrint("[");
			debugPrint(dac.getPowerDown(i, true), DEC);
			debugPrint("]\t");

			debugPrint(dac.getDACData(i), DEC);
			debugPrint("[");
			debugPrint(dac.getDACData(i, true), DEC);
			debugPrint("]\r\n");
		}
	}

  bool printSWS = 0;

private:
	Stream* stream = NULL;
  elapsedMillis initRetryTimer = -2000;
	bool isInit = false;
	bool steerOutputEnabled = 0;
	bool ch4Enabled = 0;
	//uint16_t left_center_right_DAC[3] = { 3512, 2019, 501 };
	//uint16_t left_center_right_ADS[3] = { 22812, 13150, 3501 };
	//uint8_t outputIndex = 0;
  //float adsToDacConvFactor = 6.49;  // for bench testing
  float adsToDacConvFactor = 5.875;  // for v5.0a 8320T
	const static uint8_t NUM_SWS = 4;
	uint16_t steeringWheelSensor[NUM_SWS];
	uint16_t steeringWheelSensorCenter[NUM_SWS];

	TwoWire* i2cPort;
	uint8_t dacAddr;
	uint8_t dacSetCenterIO;
	uint8_t dacDevBtn;
	MCP4728::PWR_DOWN DEFAULT_PWR_DOWN = MCP4728::PWR_DOWN::GND_500KOHM;

	//const uint16_t LOOP_TIME = 25;  //40Hz    
	uint32_t lastAdsUpdateTime;
	uint32_t currentAdsUpdateTime;
	uint32_t lastPrintTime;
	uint32_t currentPrintTime;
	uint8_t adsIndex;
	//int16_t adsOffset[4] = { 425, 212, 369, 177 };
	//elapsedMillis adsTimer;
	uint16_t sweep = 0;
	uint32_t sweepTriggerTime;
	
	template<typename M>
	inline size_t debugPrint(M _message) {
		if (stream != NULL) {
			return stream->print(_message);
		}
		return false;
	}

	template<typename N>
	inline size_t debugPrint(N _number, int16_t _format) {
		if (stream != NULL) {
			return stream->print(_number, _format);
		}
		return false;
	}
};

#endif