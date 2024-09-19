/*

Started Feb 2024 - Matt Elias
Updated Apr 2024

Used Ace repo code and adapted for AiO v4.x/5.0a RVC 100hz
- Started with code from Teensy "Nav" Ace module https://github.com/farmerbriantee/Ace/tree/master/Hardware/Ace
- Added code bits from old AIO v4 I2C firmware https://github.com/AgHardware/Boards/tree/main/Boards/TeensyModules/AIO%20Micro%20v4/Firmware/Autosteer_gps_teensy_v4
- Much many new original code written for performance monitoring, LED control, section control etc


Single Operation Logic
- Save BNO reading 60ms before next GGA/GNS (40ms after last GGA, comes from F9P pole swing test)
	- roll/heading will be ready for PANDA later
	- if no BNO, prep vars with 0xFFFF heading, 0 roll/yaw/pitch
  - for UM982 I expect the timing should be different
- Once GGA/GNS arrives (if !useDual)
	- build PANDA msg and send out
	- Otherwise if useDual, wait for heading (relposned from F9P or HPR from UM982) in main loop()

Dual Operation Logic
- if heading arrives
	- set useDual for duration of runtime
- Each time new GGA/GNS & heading arrive
	- if fix/diffsol/posvalid all good
		- calc roll from dual baseline/heading data
			- if carrsoln is not full RTK "wind down" dual roll by x0.9 each GPS update
	- Send PAOGI

Autosteer updates at 100hz but should maybe only be at the old 40hz?


Machine/Section outputs
- only supported by v5.0
- uses Wire1/I2C1 (v5.0a-c) or Wire/I2C (v5.0d) to connect to PCA9555
  - v4.x uses Wire1 to connect to ADS1115



To-do
- Adafruit NeoPixel (WS2811) library disabled interrupts which causes lost Serial data
  - this has been overriden to keep interrupts on, LEDs don't blink quite 100%
  - for v5.0d, add code for WS2812Serial library for nonblock RGB control
- send more data to ESP32, such as speed and roll correction position
- set ADS1115 single/dual according to config struct setting (saved in eeprom)
- consolidate all EEPROM addrs in one place?
    - Ethernet, Autosteer, machine
- test/fix autosteer watch dog timeout from lost comms
- write piezo class
- expand machine/PCA9555 to monitor outputs with PCA's extra input pins
- add analog PCB ID input
- use 2nd Eth jack LED for something?
- intelligently detect Eth link status?
  - Ethernet.linkStatus doesn't work until after the correct Ethernet.begin has started
  - probably need to use diff Ethernet library like AsyncUDP_Teensy41

- Testing !!!
  - pressure/current inputs should be scaled the same as old firmware, only bench tested by Matt
  - Single/IMU PANDA calcs should be the same as Ace branch
  - Dual PAOGI calcs should be the same as old I2C AIO firmware



*/