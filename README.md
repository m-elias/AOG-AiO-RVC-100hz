![AgOpenGPS](https://github.com/m-elias/AOG-AiO-RVC-100hz/blob/main/media/agopengps%20name%20logo.png)
[AOG Download](https://github.com/farmerbriantee/AgOpenGPS/releases)<br>
[AOG Forum](https://discourse.agopengps.com/)<br>
[AOG YouTube](https://youtube.com/@AgOpenGPS)

# 100hz RVC firmware for All-In-One v4.x & v5.0a
### *** alpha testing stage ***
*** currently testing/debugging missing GPS sentences/msgs ***

A Teensy 4.1 (w/Eth) firmware for the AIO PCB to run the BNO IMU and Autosteer control loop at 100hz.

Currently it supports both I2C ads1115 (like previous firmwares) and the MCP module (subject to change if field testing shows one or the other doesn’t work well). The MCP module uses a MCP6002 opamp to buffer (protect) the Teensy's analog input eliminating the need to use the slow I2C ads1115. Bench tests show that putting the ads1115 into "continous convert" mode & a faster sampling speed largely mitigates its I2C speed penalties.

At this point, to get the latest version, it's probably best to compile the source code yourself (as I expect there to be many/freq fixes/changes as testing happens) but there are also pre-compiled HEX files for v4.x and v5.0a AIO PCBs.

The latest source code is in the [RVC_100hz_mixed](https://github.com/m-elias/AOG-AiO-RVC-100hz/tree/main/RVC_100hz_mixed) folder.
I use Arduino IDE v2.3.1, Teensyduino 1.58.2, Streaming v6.1.1.

The pre-compiled HEX files are in the [root directory](https://github.com/m-elias/AOG-AiO-RVC-100hz)

### To do:
See the [notes](https://github.com/m-elias/AOG-AiO-RVC-100hz/blob/main/RVC_100hz_mixed/1_notes.ino)

### AIO v4.5 Std RVC config options
- To connect the BNO SDA/TX pin to the Teensy's serial RX pin, solder the green circled solder pads (RVC-RX5).
- Then to put the BNO into RVC mode, solder one of the two yellow circled solder pads, either on the PCB (RVC Mode) or on the BNO (PS0). If you solder the PCB's "RVC Mode" pads, then every BNO you plug in will be in RVC mode. If you solder the BNO's "PS0" pads, then only that BNO will be in RVC mode and you have the option of going back to I2C mode with a different BNO (and old Teensy firmware).
- In my testing it was not necessary to cut the red circled pad traces but if you experience unstable RVC BNO performance, you may need to cut the traces between the red circled solder pads to disconnect the BNO from the Teensy's I2C pins.
- Lastly, the 100hz RVC firmware currently supports both I2C ADS1115 and a new MCP6002 module (blue & green modules) for WAS input signal. You can try using the I2C ADS1115 without any additional changes or modules but if you wish to use the MCP6002 module you also need to solder the blue circled A0 solder pads to connect the Teensy A0 to the MCP output.
![AIO v4.5 Std RVC config](https://github.com/m-elias/AOG-AiO-RVC-100hz/blob/main/media/AIO%20v4.5%20Std%20RVC%20config%20options.jpg)

### Modified AgOpenGPS.exe
There is a modified copy of AgOpenGPS.exe in the [AOG modified for freeform msg pop-up](https://github.com/m-elias/AOG-AiO-RVC-100hz/tree/main/AOG%20modified%20for%20freeform%20msg%20pop-up) folder for testing roll corrected position output and sending a freeform pop-up message from the AIO firmware to AOG. Currently it sends the Pop-Up Msg PGN when the autosteer switch/btn is activated (see Autosteer.ino line [200](https://github.com/m-elias/AOG-AiO-RVC-100hz/blob/279e0f3a72f9fc7b0018a4bd1440960cb8f07813/RVC_100hz_mixed/Autosteer.ino#L200), 208, 223). The pop-up msg works on both AIO PCBs but the roll corrected position output is intended for the v5.0a to output GGA/VTG/RMC via its onboard RS232.
