#!/bin/bash
# camera_calibrate.sh
#
# Self-setting-up camera calibration launcher. On first run, creates a local
# venv with numpy<2 so cv_bridge works correctly (ROS 2 Humble cv_bridge is
# built against numpy 1.x; numpy 2.x breaks it).
#
# If IMAGE_TOPIC is not set, the script lists every live sensor_msgs/Image
# topic on the ROS graph and lets you pick one from an interactive menu.
#
# DEFAULTS (override via environment variables before calling):
#   SIZE          Checkerboard interior corners WxH   default: 7x5
#   SQUARE        Square side length in metres         default: 0.032 (32mm)
#   IMAGE_TOPIC   ROS image topic to subscribe to      default: interactive pick
#   CAMERA_NS     ROS camera namespace                 default: dirname(IMAGE_TOPIC)
#   ROS_DOMAIN_ID ROS domain ID                        default: 23
#
# MINIMAL USAGE (picks topic interactively):
#   ./camera_calibrate.sh
#
# NON-INTERACTIVE / SCRIPTED:
#   SIZE=9x7 SQUARE=0.030 IMAGE_TOPIC=/cam_down/image_raw \
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

# ── 5. Resolve IMAGE_TOPIC ───────────────────────────────────────────────────
# If not set via env var, query the live ROS graph for sensor_msgs/Image topics
# and present an interactive numbered menu.
if [ -z "${IMAGE_TOPIC:-}" ]; then
    echo "[calib] Scanning ROS graph for sensor_msgs/Image topics..."
    mapfile -t IMAGE_TOPICS < <(ros2 topic list -t 2>/dev/null \
        | awk '$2 == "[sensor_msgs/msg/Image]" {print $1}' \
        | sort)

    if [ ${#IMAGE_TOPICS[@]} -eq 0 ]; then
        echo "[calib] ERROR: no sensor_msgs/Image topics visible on the graph."
        echo "[calib]        Start a camera pipeline first, then re-run this script."
        exit 1
    fi

    echo "[calib] Select an image topic to calibrate (or Ctrl-C to abort):"
    PS3="[calib] > "
    select IMAGE_TOPIC in "${IMAGE_TOPICS[@]}"; do
        if [ -n "$IMAGE_TOPIC" ]; then break; fi
        echo "[calib] Invalid selection."
    done
fi

# Derive CAMERA_NS from the topic by stripping the trailing segment
# (e.g. /cam_down/image_raw → /cam_down). Override via env var if needed.
CAMERA_NS="${CAMERA_NS:-$(dirname "$IMAGE_TOPIC")}"

SIZE="${SIZE:-7x5}"
SQUARE="${SQUARE:-0.032}"

echo "[calib] Topic:    $IMAGE_TOPIC"
echo "[calib] Camera:   $CAMERA_NS"
echo "[calib] Board:    ${SIZE} @ ${SQUARE}m"

# ── 6. Launch calibrator ─────────────────────────────────────────────────────
PYTHONNOUSERSITE=1 \
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}" \
    "$VENV_DIR/bin/python3" \
    /opt/ros/humble/lib/camera_calibration/cameracalibrator \
    --size "$SIZE" --square "$SQUARE" \
    --ros-args --remap "image:=$IMAGE_TOPIC" --remap "camera:=$CAMERA_NS" || true

# ── 7. Copy calibration output to script directory ──────────────────────────
# "Commit" writes /tmp/ost.yaml; "Save" writes /tmp/calibrationdata.tar.gz
if [ ! -f /tmp/ost.yaml ] && [ -f /tmp/calibrationdata.tar.gz ]; then
    tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/ ost.yaml 2>/dev/null || true
fi

if [ -f /tmp/ost.yaml ]; then
    # Derive filename from the topic (strip leading /, replace / with _).
    # e.g. /cam_down/image_raw  →  cam_down_image_raw_calibration.yaml
    TOPIC_SLUG=$(echo "$IMAGE_TOPIC" | sed 's|^/||; s|/|_|g')
    OUT="$SCRIPT_DIR/${TOPIC_SLUG}_calibration.yaml"
    cp /tmp/ost.yaml "$OUT"
    echo "[calib] Calibration saved to $OUT"
else
    echo "[calib] No calibration output found. Did you click Save before closing?"
fi
