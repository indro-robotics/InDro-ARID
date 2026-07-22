#!/bin/bash
# in_isaac.sh - run a container alias/command from the host inside the Isaac container.
# Used by the host mirrors of the container aliases (colcon_isaac, clean_isaac, ...).
#
# Usage:
#   in_isaac.sh <alias-or-command...>     # interactive when on a TTY, else plain
#   in_isaac.sh -t <alias-or-command...>  # force a TTY

set -uo pipefail

CONTAINER=isaac_ros_dev-aarch64-container

# Allocate a TTY when we have one (live output + interactive prompts); plain -i otherwise.
FLAGS=(-i)
{ [ -t 0 ] && [ -t 1 ]; } && FLAGS=(-it)
[[ "${1:-}" == "-t" ]] && { FLAGS=(-it); shift; }

if ! docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null | grep -q true; then
    echo "ERROR: ${CONTAINER} is not running. Try: sudo systemctl start start_isaac_docker.service" >&2
    exit 1
fi

# bash -i: the container's aliases load only in an interactive shell, not a plain docker exec.
# -u admin: same uid as the supervisor and host so control service calls share Fast-DDS SHM.
cmd="$(printf '%q ' "$@")"
exec docker exec -u admin "${FLAGS[@]}" "${CONTAINER}" bash -ic "${cmd}"
