#!/bin/bash
# Submodule sync, verification, and pin-advancement tool.
#
# Usage:
#   update_isaac_submods.sh           — sync to pinned SHAs + verify (used by setup.sh)
#   update_isaac_submods.sh --update  — advance live submodules to branch tips / retag pinned ones
#                                       then report what changed so you can commit + push
set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
cd "$REPO_ROOT"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }
warn() { echo -e "  ${YELLOW}[WARN]${NC}  $*"; }
info() { echo -e "  ${YELLOW}>>>${NC} $*"; }

###############################################################################
# Submodule definitions
#
# LIVE    — indro-controlled, branch-tracked; verify SHA is on expected branch
# PINNED  — external, exact tag required; verify tag matches
###############################################################################

# Format: "path:branch"
LIVE_SUBMODULES=(
    "isaac_ros-dev/src/apriltag_cypher:cypher_v3"
    "isaac_ros-dev/src/isaac_ros_argus_camera:release-3.2"
    "local_ws/src/reset_ark_usb:main"
    "local_ws/src/uwb_drone:websocket-server"
)

# Format: "path:tag"
PINNED_SUBMODULES=(
    "isaac_ros-dev/src/realsense-ros:4.51.1"
    "isaac_ros-dev/src/px4-ros2-interface-lib:1.4.0"
    "isaac_ros-dev/src/foxglove-sdk:sdk/v0.16.3"
)

###############################################################################
# MODE: --update  (developer tool — advance pins)
###############################################################################
if [[ "${1:-}" == "--update" ]]; then
    echo "=== Updating live (branch-tracked) submodules ==="
    for entry in "${LIVE_SUBMODULES[@]}"; do
        path="${entry%%:*}"; branch="${entry##*:}"
        if [[ -d "$path" ]]; then
            info "${path} → branch ${branch}"
            (cd "$path" && git fetch origin && git checkout "$branch" && git pull origin "$branch")
            ok "${path} updated"
        else
            warn "${path} not found — skipping"
        fi
    done

    echo ""
    echo "=== Pinning tag submodules ==="
    for entry in "${PINNED_SUBMODULES[@]}"; do
        path="${entry%%:*}"; tag="${entry##*:}"
        if [[ -d "$path" ]]; then
            info "${path} → tag ${tag}"
            (cd "$path" && git fetch --all --tags && git checkout "$tag")
            ok "${path} pinned to ${tag}"
        else
            warn "${path} not found — skipping"
        fi
    done

    echo ""
    echo "=== Submodule status ==="
    git submodule status
    echo ""
    echo "If the above looks correct:"
    echo "  git add .gitmodules <changed submodule paths>"
    echo "  git commit -m 'chore: bump submodule pins'"
    echo "  git push"
    exit 0
fi

###############################################################################
# MODE: default  (sync + verify — called by setup.sh)
###############################################################################
echo "Syncing all submodules to pinned commits..."
git submodule update --init --recursive
echo ""

ERRORS=0

echo "Verifying live (branch-tracked) submodules..."
for entry in "${LIVE_SUBMODULES[@]}"; do
    path="${entry%%:*}"; expected_branch="${entry##*:}"

    if [[ ! -d "$path" ]]; then
        err "${path}: directory not found"
        (( ERRORS++ )) || true
        continue
    fi

    sha=$(git -C "$path" rev-parse HEAD)
    short="${sha:0:10}"
    git -C "$path" fetch origin --quiet 2>/dev/null || true
    remote_tip=$(git -C "$path" rev-parse "origin/${expected_branch}" 2>/dev/null || echo "")

    if [[ -z "$remote_tip" ]]; then
        warn "${path} @ ${short} — could not resolve origin/${expected_branch} (offline?)"
    elif [[ "$sha" == "$remote_tip" ]]; then
        ok "${path} @ ${short} (tip of origin/${expected_branch})"
    elif git -C "$path" merge-base --is-ancestor "$sha" "origin/${expected_branch}" 2>/dev/null; then
        ok "${path} @ ${short} (on origin/${expected_branch}, pinned behind tip)"
    else
        err "${path} @ ${short} — commit is NOT on origin/${expected_branch}"
        err "  Run: scripts/update_isaac_submods.sh --update"
        (( ERRORS++ )) || true
    fi
done

echo ""
echo "Verifying pinned (tag) submodules..."
for entry in "${PINNED_SUBMODULES[@]}"; do
    path="${entry%%:*}"; expected_tag="${entry##*:}"

    if [[ ! -d "$path" ]]; then
        err "${path}: directory not found"
        (( ERRORS++ )) || true
        continue
    fi

    sha=$(git -C "$path" rev-parse HEAD)
    short="${sha:0:10}"
    actual_tag=$(git -C "$path" tag --points-at HEAD 2>/dev/null | tr '\n' ' ' | xargs)

    if ! echo "$actual_tag" | grep -qw "$expected_tag"; then
        git -C "$path" fetch origin --tags --quiet 2>/dev/null || true
        actual_tag=$(git -C "$path" tag --points-at HEAD 2>/dev/null | tr '\n' ' ' | xargs)
    fi

    if echo "$actual_tag" | grep -qw "$expected_tag"; then
        ok "${path} @ ${short} (tag: ${expected_tag})"
    else
        err "${path} @ ${short} — expected tag '${expected_tag}', found: '${actual_tag:-none}'"
        err "  Run: scripts/update_isaac_submods.sh --update"
        (( ERRORS++ )) || true
    fi
done

echo ""
if [[ $ERRORS -gt 0 ]]; then
    err "${ERRORS} submodule(s) failed verification — aborting setup."
    exit 1
fi

echo "All submodules verified."
