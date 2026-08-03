#!/bin/bash
# Submodule sync and verification, run by setup.sh. With --update, advances the live submodules
# to their branch tips and re-checks out the pinned ones instead of verifying.
set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
cd "$REPO_ROOT"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }
warn() { echo -e "  ${YELLOW}[WARN]${NC}  $*"; }
info() { echo -e "  ${YELLOW}>>>${NC} $*"; }

# "path:branch". InDro-controlled: HEAD must be on that branch, at or behind the tip.
LIVE_SUBMODULES=(
    "isaac_ros-dev/src/px4_msgs:release/1.15"
    "local_ws/auxiliary/PX4-Autopilot:PX4-InDro"
    "isaac_ros-dev/src/realsense-ros:v4.51.1"
    "isaac_ros-dev/src/isaac_ros_visual_slam:v3.2-14"
)

# "path:tag". External: HEAD must sit exactly on that tag.
PINNED_SUBMODULES=(
    "isaac_ros-dev/src/isaac_ros_common:v3.2-14"
    "isaac_ros-dev/src/isaac_ros_nitros:v3.2-14"
    "isaac_ros-dev/src/isaac_ros_image_pipeline:v3.2-14"
    "isaac_ros-dev/src/px4-ros2-interface-lib:1.4.0"
    "local_ws/src/rslidar_sdk:v1.5.19"
    "local_ws/src/rslidar_msg:v1.5.10"
)

if [[ "${1:-}" == "--update" ]]; then
    echo "=== Updating live (branch-tracked) submodules ==="
    for entry in "${LIVE_SUBMODULES[@]}"; do
        path="${entry%%:*}"; branch="${entry##*:}"
        if [[ -d "$path" ]]; then
            info "${path} -> branch ${branch}"
            # -B off the remote ref: a bare `git checkout <name>` prefers a same-named TAG.
            (cd "$path" && git fetch origin && git checkout -B "$branch" "origin/$branch")
            ok "${path} updated"
        else
            warn "${path} not found - skipping"
        fi
    done

    echo ""
    echo "=== Pinning tag submodules ==="
    for entry in "${PINNED_SUBMODULES[@]}"; do
        path="${entry%%:*}"; tag="${entry##*:}"
        if [[ -d "$path" ]]; then
            info "${path} -> tag ${tag}"
            (cd "$path" && git fetch --all --tags && git checkout "$tag")
            ok "${path} pinned to ${tag}"
        else
            warn "${path} not found - skipping"
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

echo "Syncing all submodules to pinned commits..."
git submodule sync --recursive
git submodule update --init --recursive
echo ""

# `git submodule update` is a no-op when HEAD already matches the pin, even if every file in the
# working tree was deleted.
echo "Ensuring submodule working trees are populated..."
while IFS= read -r path; do
    [[ -d "$path" ]] || continue
    [[ "$(git -C "$path" ls-tree -r --name-only HEAD 2>/dev/null | wc -l)" -gt 0 ]] || continue
    if [[ -z "$(find "$path" -type f -not -path '*/.git/*' -print -quit 2>/dev/null)" ]]; then
        warn "${path}: working tree empty (files were deleted) - restoring from HEAD"
        git -C "$path" reset --hard HEAD >/dev/null 2>&1 || true
        ok "${path}: restored ($(git -C "$path" ls-files 2>/dev/null | wc -l) files)"
    fi
done < <(git config -f .gitmodules --get-regexp 'path' | awk '{print $2}')
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
        warn "${path} @ ${short} - could not resolve origin/${expected_branch} (offline?)"
    elif [[ "$sha" == "$remote_tip" ]]; then
        ok "${path} @ ${short} (tip of origin/${expected_branch})"
    elif git -C "$path" merge-base --is-ancestor "$sha" "origin/${expected_branch}" 2>/dev/null; then
        ok "${path} @ ${short} (on origin/${expected_branch}, pinned behind tip)"
    else
        err "${path} @ ${short} - commit is NOT on origin/${expected_branch}"
        err "  Run: scripts/update_submods.sh --update"
        (( ERRORS++ )) || true
    fi
done

echo ""
echo "Pinning tag submodules to their exact tags (corrects any drift)..."
for entry in "${PINNED_SUBMODULES[@]}"; do
    path="${entry%%:*}"; expected_tag="${entry##*:}"

    if [[ ! -d "$path" ]]; then
        err "${path}: directory not found"
        (( ERRORS++ )) || true
        continue
    fi

    if ! git -C "$path" tag --points-at HEAD 2>/dev/null | grep -qw "$expected_tag"; then
        git -C "$path" fetch origin --tags --quiet 2>/dev/null || true
        git -C "$path" checkout --quiet "$expected_tag" 2>/dev/null || true
    fi

    sha=$(git -C "$path" rev-parse HEAD)
    short="${sha:0:10}"
    actual_tag=$(git -C "$path" tag --points-at HEAD 2>/dev/null | tr '\n' ' ' | xargs)

    if echo "$actual_tag" | grep -qw "$expected_tag"; then
        ok "${path} @ ${short} (tag: ${expected_tag})"
    else
        err "${path} @ ${short} - could not pin to tag '${expected_tag}', found: '${actual_tag:-none}'"
        err "  Run: scripts/update_submods.sh --update"
        (( ERRORS++ )) || true
    fi
done

echo ""
if [[ $ERRORS -gt 0 ]]; then
    err "${ERRORS} submodule(s) failed verification - aborting setup."
    exit 1
fi

echo "All submodules verified."
