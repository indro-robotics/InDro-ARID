#!/bin/bash
# in_isaac.sh - run a container alias or command from the host inside the Isaac container.

set -uo pipefail

CONTAINER=isaac_ros_dev-aarch64-container

# `docker exec -it` fails outright when stdin or stdout is a pipe, so -it requires a real TTY
# on both.
FLAGS=(-i)
{ [ -t 0 ] && [ -t 1 ]; } && FLAGS=(-it)
[[ "${1:-}" == "-t" ]] && shift

if ! docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null | grep -q true; then
    echo "ERROR: ${CONTAINER} is not running. Try: sudo systemctl start start_isaac_docker.service" >&2
    exit 1
fi

# bash -i: the container's aliases load only in an interactive shell, not a plain docker exec.
# -u admin: same uid as the supervisor and the host, so service calls share Fast-DDS SHM.
#
# Killing the docker exec CLIENT does not kill the in-container process: a host Ctrl+C or SSH
# drop orphans a long-running alias (initialize) inside the container. SIGKILL of the client
# is uncoverable.
cmd="$(printf '%q ' "$@")"
TOKEN="aridshim_$$_${RANDOM}"
reap_container_run() {
    # The container runs --pid=host, so /proc inside it holds both the in-container bash -ic and
    # the host docker client. The reap_by_token marker stops the sweep matching itself.
    # Group-kill only a verified leader: a non-leader's PID can equal an innocent group's PGID.
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
    # The pty delivers Ctrl+C inward, and a client death closes it (SIGHUP to the foreground
    # group), so this path needs no token sweep.
    exec docker exec -u admin "${FLAGS[@]}" "${CONTAINER}" bash -ic "${cmd}"
fi
# setsid: a bare `bash -ic` ignores a direct INT and is not reliably a group leader, so without
# its own session the token would not identify a killable process group.
# The trap disarms itself first: the sweep signals the shim's own group, and a nested trap would
# double the reap.
rc=""
trap 'trap "" INT TERM HUP; reap_container_run; exit "${rc:-130}"' INT
trap 'trap "" INT TERM HUP; reap_container_run; exit "${rc:-143}"' TERM HUP
docker exec -u admin "${FLAGS[@]}" "${CONTAINER}" setsid -w bash -ic ": ${TOKEN}; ${cmd}"
rc=$?
trap - INT TERM HUP
exit "${rc}"
