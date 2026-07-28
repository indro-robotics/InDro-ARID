#!/bin/bash
# in_isaac.sh - run a container alias/command from the host inside the Isaac container.
# Used by the host mirrors of the container aliases (colcon_isaac, clean_isaac, ...).
#
# Usage:
#   in_isaac.sh <alias-or-command...>     # interactive when on a TTY, else plain
#   in_isaac.sh -t <alias-or-command...>  # force a TTY

set -uo pipefail

CONTAINER=isaac_ros_dev-aarch64-container

# Allocate a TTY when we have one; plain -i otherwise. -t is honored only with a
# real TTY (docker exec -it from a pipe hard-fails) - else fall back to the shim.
FLAGS=(-i)
{ [ -t 0 ] && [ -t 1 ]; } && FLAGS=(-it)
[[ "${1:-}" == "-t" ]] && shift

if ! docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null | grep -q true; then
    echo "ERROR: ${CONTAINER} is not running. Try: sudo systemctl start start_isaac_docker.service" >&2
    exit 1
fi

# bash -i: the container's aliases load only in an interactive shell, not a plain docker exec.
# -u admin: same uid as the supervisor and host so control service calls share Fast-DDS SHM.
#
# Signal forwarding: killing the docker-exec CLIENT does NOT kill the in-container
# process, so a host Ctrl+C/SSH drop would orphan a long-running alias (initialize)
# inside the container. The run is tagged with a unique token and started as a
# session leader; on INT/TERM/HUP the trap reaps the whole in-container process
# group by token. SIGKILL of the client remains uncoverable.
cmd="$(printf '%q ' "$@")"
TOKEN="aridshim_$$_${RANDOM}"
reap_container_run() {
    # Sweep the tagged run (in-container bash -ic + host docker client, both visible
    # via --pid=host); the reap_by_token marker excludes the reaper itself. Group-kill
    # only verified leaders (/proc/stat pgid == pid) - a non-leader's PID may equal an
    # innocent group's PGID.
    local sweep
    sweep='
        tok="$1"; sig="$2"
        for d in /proc/[0-9]*; do
            p=${d#/proc/}
            [ "$p" = "$$" ] && continue
            c=$(tr "\0" " " < "$d/cmdline" 2>/dev/null) || continue
            case "$c" in *"reap_by_token"*) continue ;; esac
            case "$c" in *"$tok"*)
                rest=$(cat "$d/stat" 2>/dev/null) || continue
                rest=${rest##*) }; set -- $rest; pg=${3:-}
                if [ "$pg" = "$p" ]; then kill "-$sig" -- "-$p" 2>/dev/null
                else kill "-$sig" "$p" 2>/dev/null; fi ;;
            esac
        done
        true reap_by_token'
    docker exec "${CONTAINER}" bash -c "${sweep}" _ "${TOKEN}" INT 2>/dev/null
    sleep 3
    docker exec "${CONTAINER}" bash -c "${sweep}" _ "${TOKEN}" TERM 2>/dev/null
}
if [[ "${FLAGS[*]}" == *t* ]]; then
    # TTY path: the pty delivers Ctrl+C inward natively, and a client death closes the
    # pty -> SIGHUP to the foreground group. No shim needed.
    exec docker exec -u admin "${FLAGS[@]}" "${CONTAINER}" bash -ic "${cmd}"
fi
# Non-TTY path: start the run as its OWN SESSION so the token identifies a real process
# group the reaper can kill wholesale (a bare bash -ic ignores a direct INT and is not
# reliably a group leader).
# Disarm traps first (the sweep signals the shim's own group - a nested trap would double
# the reap); a captured rc wins over the signal code.
rc=""
trap 'trap "" INT TERM HUP; reap_container_run; exit "${rc:-130}"' INT
trap 'trap "" INT TERM HUP; reap_container_run; exit "${rc:-143}"' TERM HUP
docker exec -u admin "${FLAGS[@]}" "${CONTAINER}" setsid -w bash -ic ": ${TOKEN}; ${cmd}"
rc=$?
trap - INT TERM HUP
exit "${rc}"
