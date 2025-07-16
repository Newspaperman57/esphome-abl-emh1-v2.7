# esphome-abl-emh1
Esphome component for communication with ABL Wallbox eMH1 with the (older?) ABL SURSUM EVCC v2.7 controller

This is a fork of the original project which uses a slightly different protocol: https://github.com/jrv/esphome-abl-emh1

It reads the current on 3 phases and allows you to set the max current.
There is also a switch to enable/disable the charger.

Some more output (like the serial number) is available in hidden entities.

### My hardware
- ESP-wroom 32 (esp32dev)
- ESP GPIO pin 5 is flow-control 
- SN75176 to convert serial to RS485
- 5V Din-rail power supply (optional, you can also get 12v power from conn X10 and convert that to 5V)
- Esp Ground connected to the GND pin from connector X10 on the ABL eMH1 circuit board. This helps avoiding noise on the RS485.
- RJ45 plug: pin 1 and 2 connected to SN75176, no other pins connected

### More info, questions

Check out this thread on the Home Assistant community forum:
https://community.home-assistant.io/t/connecting-the-abl-emh1-ev-charger-as-esphome-component

### Disclaimer
Using this component requires you to connect consumer electronics
inside your ABL eMH1 Wallbox. There's also ~380V in that box, so
please make sure you know what your doing. 

Connecting this to your electricity network and your car is your own
responsibility.
