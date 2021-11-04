# Introduction
The scope of this and other projects is to ease deployment and debugging for devices that may not be physically accessible during development phase.

The device contacts an OTA Server once after startup to check for updates and self-flash if one is available.
It bridges debugging output to a terminal via a bridge-server (which in my case is included in the same microservice as the OTA-Server).

Linker file modifications are used to not have to worry about renaming binary output after compilation.
The idea is to simply drop the generated `firmware.bin` onto a filesystem the OTA-Server can access.
The OTA Server then can read the raw data at a predefined location and store device and version data in a db or whatever deemed suitable

This project uses a yet to open source PCB to interact with various sensors such as:
- BME280 for outside temperature, humidity, pressure
- Dallas DS18B20 for ambient temperature
- ACS712 for isolated current measurements
- Solar Charge Controller via RS485

Codebase for Solar Charge Controller (EPEVER Tracer) is from [@tekk](https://github.com/tekk):
https://github.com/tekk/Tracer-RS485-Modbus-Blynk-V2

All my devices are designed to work from 5 .. 28 VDC supply (30A absolute max.) and I have taken care to keep the power consumption to a minimum.
If this is something for you, stay tuned for schematics and layout data that I will soon provide here.

# Usage

More detailed info if you are seriously inclined to already use this can be found [here](include/README.md).

Have fun and let me know what you like/dislike.
