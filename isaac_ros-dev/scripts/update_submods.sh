#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Change to the root directory of the git repository (one level up from scripts)
cd "$SCRIPT_DIR/.." || exit

# Update all submodules to their specified branches
git submodule foreach 'git checkout $(git config -f $toplevel/.gitmodules submodule.$name.branch || echo master)'

# Checkout realsense-ros to the specific tag
if [ -d "src/realsense-ros" ]; then
    cd src/realsense-ros || exit
    git fetch --all --tags
    git checkout 4.51.1
    cd ../..
else
    echo "Warning: src/realsense-ros directory not found"
fi

# Checkout px4-ros2-interface-lib to the specific tag
if [ -d "src/px4-ros2-interface-lib" ]; then
    cd src/px4-ros2-interface-lib || exit
    git fetch --all --tags
    git checkout 1.4.0
    cd ../..
else
    echo "Warning: src/px4-ros2-interface-lib directory not found"
fi

# Checkout foxglove-sdk to the specific tag
if [ -d "src/ros-foxglove-bridge" ]; then
    cd src/ros-foxglove-bridge || exit
    git fetch --all --tags
    git checkout sdk/v0.16.3
    cd ../..
else
    echo "Warning: src/ros-foxglove-bridge directory not found"
fi

# Check if any submodules were updated
if [ -n "$(git status --porcelain)" ]; then
    echo "Submodules updated. Changes detected in the following submodules:"
    git status --porcelain | grep "src/" | awk '{print $2}'
    echo "realsense-ros is now at tag 4.51.1 (if it exists)"
    echo "px4-ros2-interface-lib is now at tag 1.4.0 (if it exists)"
    echo "foxglove-sdk is now at tag sdk/v0.16.3 (if it exists)"
else
    echo "No changes detected in submodules."
fi
