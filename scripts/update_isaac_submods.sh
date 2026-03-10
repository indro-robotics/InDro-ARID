#!/bin/bash
# Sync all submodules to their pinned SHAs and verify they are at the expected
# commits/tags. Called by setup.sh — does NOT pull or advance any pins.
#
# To intentionally advance a submodule pin, use:
#   scripts/dev/update_submodule_pins.sh
set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
cd "$REPO_ROOT"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }
warn() { echo -e "  ${YELLOW}[WARN]${NC}  $*"; }

###############################################################################
# STEP 1: sync all submodules to their pinned SHAs
###############################################################################
echo "Syncing all submodules to pinned commits..."
git submodule update --init --recursive
echo ""

###############################################################################
# STEP 2: verify
#
# Two categories:
#   LIVE     — indro-controlled, branch-tracked; verify SHA is on expected branch
#   PINNED   — external, exact tag required;     verify tag matches
#
# Format: "path:expected_ref"
###############################################################################

# Branch-tracked live submodules (no exact tag, just must be on the branch)
LIVE_SUBMODULES=(
    "isaac_ros-dev/src/apriltag_cypher:cypher_v3"
    "isaac_ros-dev/src/isaac_ros_argus_camera:release-3.2"
    "local_ws/src/reset_ark_usb:main"
    "local_ws/src/uwb_drone:websocket-server"
)

# External submodules that must be at a specific tag
PINNED_SUBMODULES=(
    "isaac_ros-dev/src/realsense-ros:4.51.1"
    "isaac_ros-dev/src/px4-ros2-interface-lib:1.4.0"
    "isaac_ros-dev/src/foxglove-sdk:sdk/v0.16.3"
)

ERRORS=0

echo "Verifying live (branch-tracked) submodules..."
for entry in "${LIVE_SUBMODULES[@]}"; do
    path="${entry%%:*}"
    expected_branch="${entry##*:}"

    if [[ ! -d "$path" ]]; then
        err "${path}: directory not found — was 'git submodule update --init --recursive' run?"
        (( ERRORS++ )) || true
        continue
    fi

    sha=$(git -C "$path" rev-parse HEAD)
    short="${sha:0:10}"

    # Fetch quietly so remote-tracking refs are current
    git -C "$path" fetch origin --quiet 2>/dev/null || true

    remote_tip=$(git -C "$path" rev-parse "origin/${expected_branch}" 2>/dev/null || echo "")

    if [[ -z "$remote_tip" ]]; then
        warn "${path} @ ${short} — could not resolve origin/${expected_branch} (offline?)"
    elif [[ "$sha" == "$remote_tip" ]]; then
        ok "${path} @ ${short} (tip of origin/${expected_branch})"
    else
        # SHA is not the branch tip — it's an older pinned commit on that branch.
        # Verify the commit at least exists on the branch.
        if git -C "$path" merge-base --is-ancestor "$sha" "origin/${expected_branch}" 2>/dev/null; then
            ok "${path} @ ${short} (on origin/${expected_branch}, pinned behind tip)"
        else
            err "${path} @ ${short} — commit is NOT on origin/${expected_branch}"
            err "  Expected branch: ${expected_branch}"
            err "  Run: scripts/dev/update_submodule_pins.sh"
            (( ERRORS++ )) || true
        fi
    fi
done

echo ""
echo "Verifying pinned (tag) submodules..."
for entry in "${PINNED_SUBMODULES[@]}"; do
    path="${entry%%:*}"
    expected_tag="${entry##*:}"

    if [[ ! -d "$path" ]]; then
        err "${path}: directory not found"
        (( ERRORS++ )) || true
        continue
    fi

    sha=$(git -C "$path" rev-parse HEAD)
    short="${sha:0:10}"

    # Check local tags first, fetch if not found
    actual_tag=$(git -C "$path" tag --points-at HEAD 2>/dev/null | tr '\n' ' ' | xargs)

    if echo "$actual_tag" | grep -qw "$expected_tag"; then
        ok "${path} @ ${short} (tag: ${expected_tag})"
    else
        # Tag might not be fetched locally — try fetching tags
        git -C "$path" fetch origin --tags --quiet 2>/dev/null || true
        actual_tag=$(git -C "$path" tag --points-at HEAD 2>/dev/null | tr '\n' ' ' | xargs)

        if echo "$actual_tag" | grep -qw "$expected_tag"; then
            ok "${path} @ ${short} (tag: ${expected_tag})"
        else
            err "${path} @ ${short} — expected tag '${expected_tag}', found: '${actual_tag:-none}'"
            err "  Run: scripts/dev/update_submodule_pins.sh"
            (( ERRORS++ )) || true
        fi
    fi
done

echo ""
if [[ $ERRORS -gt 0 ]]; then
    err "${ERRORS} submodule(s) failed verification — aborting setup."
    exit 1
fi

echo "All submodules verified."
