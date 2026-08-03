#!/bin/bash
# Power-cycle the ARK PAB USB hub (port 1-2) and gpiochip0 line 85, the standalone USB3 port.
# The cycle reboots the FMU and drops the RealSense cameras off the bus. Never run it in flight.
set -e
sudo uhubctl -l 1-2 -a off
sudo uhubctl -l 1-2 -a on
sudo gpioset --drive=open-drain --mode=time -s 1 gpiochip0 85=0
