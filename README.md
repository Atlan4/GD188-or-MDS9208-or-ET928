firmware v0.78

An alternative firmware has been created for the above measuring devices.
It has only been tested on GD188.

Important note: I do not recommend overwriting the original uP. It is not certain that your measuring chip uses the same instructions as the GD188 that I have. Next week I plan to try the modification on ETxxx, but for that I need to order a completely new uP that I will resolder and upload the alternative firmware to.

If you want to go back, it would be a good idea to make a backup of the 24c256 eeprom on the uP, and use the new uP for experiments and keep the original one with the official firmware.

No one guarantees that incorrect commands will not damage the measuring chip.

The ORIGINAL firmware is not available!!!! Do everything at your own risk.

all uP pins have been identified. PB12 aka RM1 should trigger the calibration of the measurement chip. This has not been tested nor is it implemented in the custom firmware.

I also recommend making a backup of the eeprom for the measurement chip, it contains calibration values.
