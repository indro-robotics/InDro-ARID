#!/bin/bash
# Power-cycle the ARK PAB USB hub (port 1-2) and pulse the FMU reset line on gpiochip0 line 85.
set -e
sudo uhubctl -l 1-2 -a off
sudo uhubctl -l 1-2 -a on
sudo gpioset --drive=open-drain --mode=time -s 1 gpiochip0 85=0
