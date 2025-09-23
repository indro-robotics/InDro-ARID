#!/bin/bash

# Restart udev daemon
sudo service udev restart

# Source setup so we can ros2 immediately
source install/setup.bash

$@