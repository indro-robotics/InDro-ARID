#!/bin/bash
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

# All submodules at their recorded (pinned) SHAs
echo "Syncing all submodules to pinned versions..."
git submodule update --init --recursive
echo "Submodules synced."

# Pull latest only for indro-controlled repos
LIVE_SUBMODULES=(
    "isaac_ros-dev/src/apriltag_cypher"
    "isaac_ros-dev/src/isaac_ros_argus_camera"
    "local_ws/src/reset_ark_usb"
    "local_ws/src/uwb_drone"
)

echo ""
echo "Pulling latest for indro-controlled submodules..."
for path in "${LIVE_SUBMODULES[@]}"; do
    if [[ -d "$path" ]]; then
        echo "  -> $path"
        (cd "$path" && git pull)
    else
        echo "  [SKIP] $path not found"
    fi
done

echo ""
echo "Submodule update complete."
