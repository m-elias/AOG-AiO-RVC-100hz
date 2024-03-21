/*

Started Feb 2024 - Matt Elias

Used Ace repo code and adapted for AiO v4.x/5.0a RVC 100hz
- Started with code from Teensy "Nav" Ace module https://github.com/farmerbriantee/Ace/tree/master/Hardware/Ace
- Added code bits from old AIO v4 I2C firmware https://github.com/AgHardware/Boards/tree/main/Boards/TeensyModules/AIO%20Micro%20v4/Firmware/Autosteer_gps_teensy_v4
- Much many new original code written for performance monitoring, LED control, section control etc

Single
- Save BNO reading 60ms before next GGA/GNS (40ms after last GGA, comes from F9P pole swing test)
	- roll/heading will be ready for PANDA later
	- if no BNO, prep vars with 0xFFFF heading, 0 roll/yaw/pitch
- Once GGA/GNS arrives (if !useDual)
	- build PANDA msg and send out
	- Otherwise if useDual, wait for relposned in main loop()

Dual
- if relposned arrives
	- set useDual for duration of runtime
- Each time new GGA/GNS & relposned arrive
	- if fix/diffsol/posvalid all good
		- calc roll from dual baseline/relposD
			- if carrsoln is not full RTK "wind down" dual roll by x0.9 each GPS update
	- Send paogi

Machine/Section outputs
- only supported by v5.0a


To-do
- consolidate all EEPROM addrs in one place?
    - Ethernet, Autosteer, machine
- test/fix autosteer watch dog timeout from lost comms
- write piezo class
- expand machine/PCA9555 to monitor output pins with input pins
- add analog PCB ID input
- use 2nd Eth jack LED for something?

- Testing !!!
  - pressure/current inputs should be scaled the same as old firmware, only bench tested by Matt
  - Single/IMU PANDA calcs should be the same as Ace branch
  - Dual PAOGI calcs should be the same as old I2C AIO firmware



*/