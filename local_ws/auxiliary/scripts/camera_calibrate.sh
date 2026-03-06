#!/bin/bash
# camera_calibrate.sh
#
# Self-setting-up camera calibration launcher for the cypher_argus pipeline.
# On first run, creates a local venv with numpy<2 so cv_bridge works correctly
# (ROS2 Humble cv_bridge is built against numpy 1.x; numpy 2.x breaks it).
#
# DEFAULTS (override via environment variables before calling):
#   SIZE          Checkerboard interior corners WxH   default: 7x5
#   SQUARE        Square side length in metres         default: 0.032 (32mm)
#   IMAGE_TOPIC   ROS image topic to subscribe to      default: /cam_down/image_raw
#   CAMERA_NS     ROS camera namespace                 default: /cam_down
#   ROS_DOMAIN_ID ROS domain ID                        default: 23
#
# MINIMAL USAGE (all defaults):
#   ./camera_calibrate.sh
#
# CUSTOM USAGE:
#   SIZE=9x7 SQUARE=0.030 IMAGE_TOPIC=/cam_front/image_raw CAMERA_NS=/cam_front \
#       ./camera_calibrate.sh

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
VENV_DIR="$SCRIPT_DIR/calib_env"

# ── 1. Create venv if missing ────────────────────────────────────────────────
if [ ! -f "$VENV_DIR/bin/python3" ]; then
    echo "[calib] Creating venv at $VENV_DIR ..."
    python3 -m venv "$VENV_DIR" --system-site-packages
fi

# ── 2. Ensure numpy < 2 is installed in the venv ────────────────────────────
NUMPY_MAJOR=$(PYTHONNOUSERSITE=1 "$VENV_DIR/bin/python3" \
    -c "import numpy; print(numpy.__version__.split('.')[0])" 2>/dev/null || echo "0")
if [ "$NUMPY_MAJOR" -ge "2" ] || [ "$NUMPY_MAJOR" -eq "0" ]; then
    echo "[calib] Installing numpy<2 into venv ..."
    PYTHONNOUSERSITE=1 "$VENV_DIR/bin/pip" install "numpy<2" -q
fi

# ── 3. Auto-detect DISPLAY if not set ───────────────────────────────────────
if [ -z "$DISPLAY" ]; then
    X11_SOCKET=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -1)
    if [ -n "$X11_SOCKET" ]; then
        export DISPLAY=":$(basename "$X11_SOCKET" | sed 's/X//')"
    fi
fi
echo "[calib] DISPLAY=$DISPLAY  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-23}"

# ── 4. Source ROS2 ───────────────────────────────────────────────────────────
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# ── 5. Launch calibrator ─────────────────────────────────────────────────────
SIZE="${SIZE:-7x5}"
SQUARE="${SQUARE:-0.032}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/cam_down/image_raw}"
CAMERA_NS="${CAMERA_NS:-/cam_down}"

PYTHONNOUSERSITE=1 \
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}" \
    "$VENV_DIR/bin/python3" \
    /opt/ros/humble/lib/camera_calibration/cameracalibrator \
    --size "$SIZE" --square "$SQUARE" \
    --ros-args --remap "image:=$IMAGE_TOPIC" --remap "camera:=$CAMERA_NS" || true

# ── 6. Copy calibration output to script directory ───────────────────────────
# "Commit" writes /tmp/ost.yaml; "Save" writes /tmp/calibrationdata.tar.gz
if [ ! -f /tmp/ost.yaml ] && [ -f /tmp/calibrationdata.tar.gz ]; then
    tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/ ost.yaml 2>/dev/null || true
fi

if [ -f /tmp/ost.yaml ]; then
    i=0
    while [ -f "$SCRIPT_DIR/calibration_${i}.yaml" ]; do
        i=$((i + 1))
    done
    cp /tmp/ost.yaml "$SCRIPT_DIR/calibration_${i}.yaml"
    echo "[calib] Calibration saved to $SCRIPT_DIR/calibration_${i}.yaml"
else
    echo "[calib] No calibration output found — did you click Save before closing?"
fi
