#!/bin/bash
# Developer tool: advance submodule pins to their latest desired versions.
#
# Run this manually when you want to update a submodule (new branch tip or new tag).
# After running:
#   git submodule status          # review what changed
#   git add <changed submodule paths>
#   git commit -m "chore: bump <name> to <version>"
#   git push
#
# DO NOT run this from setup.sh — setup.sh uses update_isaac_submods.sh instead.
set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
cd "$REPO_ROOT"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }
info() { echo -e "  ${YELLOW}>>>${NC} $*"; }

###############################################################################
# Live branch-tracked submodules — pull latest from their branch
# Format: "path:branch"
###############################################################################
LIVE_SUBMODULES=(
    "isaac_ros-dev/src/apriltag_cypher:cypher_v3"
    "isaac_ros-dev/src/isaac_ros_argus_camera:release-3.2"
    "local_ws/src/reset_ark_usb:main"
    "local_ws/src/uwb_drone:websocket-server"
)

###############################################################################
# Tag-pinned submodules — fetch all tags and checkout the specific tag
# Format: "path:tag"
###############################################################################
PINNED_TAGS=(
    "isaac_ros-dev/src/realsense-ros:4.51.1"
    "isaac_ros-dev/src/px4-ros2-interface-lib:1.4.0"
    "isaac_ros-dev/src/foxglove-sdk:sdk/v0.16.3"
)

echo "=== Updating live (branch-tracked) submodules ==="
for entry in "${LIVE_SUBMODULES[@]}"; do
    path="${entry%%:*}"
    branch="${entry##*:}"
    if [[ -d "$path" ]]; then
        info "${path} → branch ${branch}"
        (cd "$path" && git fetch origin && git checkout "$branch" && git pull origin "$branch")
        ok "${path} updated"
    else
        echo "  [SKIP] ${path} not found"
    fi
done

echo ""
echo "=== Pinning tag submodules ==="
for entry in "${PINNED_TAGS[@]}"; do
    path="${entry%%:*}"
    tag="${entry##*:}"
    if [[ -d "$path" ]]; then
        info "${path} → tag ${tag}"
        (cd "$path" && git fetch --all --tags && git checkout "$tag")
        ok "${path} pinned to ${tag}"
    else
        echo "  [SKIP] ${path} not found"
    fi
done

echo ""
echo "=== Submodule status after update ==="
git submodule status

echo ""
echo "If the above looks correct, commit the updated pins:"
echo "  git add .gitmodules <changed submodule paths>"
echo "  git commit -m 'chore: bump submodule pins'"
echo "  git push"
