# OGNbase

### Groundstation for Open Glider Network with ESP32

The aim of this project is to create a simple base station for the OGN network. The SoftRF project was used as the base. Thanks to Linar Yusupov for this work!

A TTGO T3S3 or LORA32 "paxcounter", or a T-Beam, board is used as hardware, which has an ESP32 processor, Wifi, a radio chip, and (in the T-Beam) a GNSS module.  Since the hardware uses very low power, it can be operated very easily on a battery with a small solar panel.

The hardware does not need another computer (Raspberry) - it sends the APRS messages directly into the Open Glider network.

OGNbase also offers the "relay" mode of operation, where the "remote" station is located at a high point for good reception of signals from aircraft, and the "base" station is located where there is internet access.  The two stations communicate via radio. 

There are also a few drawbacks relative to the traditional OGN receivers. Only one protocol can be decoded at the same time.  At the moment the second protocol has no function.  Only messages in the FLARM protocol (whether old or new) are decoded, messages in the OGNTP protocol are ignored. 

Both source code and compiled binaries are available here.  For full documentation see the [documentation file](https://github.com/moshe-braner/Open-Glider-Network-Groundstation/blob/main/ognbase/documentation/documentation.txt).  For discussions join the [SoftRF Community](https://groups.google.com/g/softrf_community).

New from version MB136:  Supports the new (March 2024) radio protocol.  Can get exact time from NTP - GNSS not needed.  Added bit error correction to increase reporting range and frequency.  Allow setting the web server port - can then access it remotely.  Detect and use OLED display on either I2C port.
