#!/bin/bash
set -euo pipefail

# Always start from the superproject root (workspaces)
cd "$(git rev-parse --show-toplevel)"

# 1) Put each submodule on its configured branch or recorded SHA
git submodule foreach '
  branch=$(git config -f "$toplevel/.gitmodules" submodule.$name.branch || echo "");
  if [ -n "$branch" ]; then
    git checkout "$branch";
  else
    git checkout "$sha1";
  fi
'

# Helper: checkout a specific tag in a named submodule, wherever it lives
checkout_tag_if_exists() {
  local module_name="$1"   # e.g. realsense-ros
  local tag="$2"           # e.g. 4.51.1

  # Find the submodule path from .gitmodules
  local path
  path=$(git config -f .gitmodules --get "submodule.${module_name}.path" || true)
  if [ -z "$path" ]; then
    # Fallback: search by leaf dir name
    path=$(git config -f .gitmodules --get-regexp '^submodule\..*\.path$' \
      | awk '{print $2}' \
      | grep "/${module_name}$" || true)
  fi

  if [ -z "$path" ] || [ ! -d "$path" ]; then
    echo "Warning: submodule '${module_name}' directory not found"
    return
  fi

  (
    cd "$path"
    git fetch --all --tags
    git checkout "$tag"
  )
}

# 2) Tag-specific submodules, regardless of whether they are under local_ws/src or isaac_ros-dev/src
checkout_tag_if_exists "realsense-ros" "4.51.1"
checkout_tag_if_exists "px4-ros2-interface-lib" "1.4.0"
checkout_tag_if_exists "foxglove-sdk" "sdk/v0.16.3"

# 3) Report changes
if [ -n "$(git status --porcelain)" ]; then
  echo "Submodules updated. Changes detected in the following submodules:"
  git status --porcelain | awk '{print $2}' | grep '^.*src/' || true
  echo "realsense-ros is now at tag 4.51.1 (if it exists)"
  echo "px4-ros2-interface-lib is now at tag 1.4.0 (if it exists)"
  echo "foxglove-sdk is now at tag sdk/v0.16.3 (if it exists)"
else
  echo "No changes detected in submodules."
fi
