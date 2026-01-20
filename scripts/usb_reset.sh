#!/bin/bash
sudo uhubctl -l 1-2 -a off
sudo uhubctl -l 1-2 -a on
sudo gpioset --drive=open-drain --mode=time -s 1 gpiochip0 85=0