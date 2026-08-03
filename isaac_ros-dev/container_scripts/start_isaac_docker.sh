#!/bin/bash
# Starts the existing Isaac container. Run by start_isaac_docker.service ExecStart at boot
# and by the `start_isaac` alias. Every failure this handles exits 0: the unit is
# Restart=on-failure, so a non-zero exit restart-loops instead of stopping.
set -e

CONTAINER=isaac_ros_dev-aarch64-container

if ! docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
    echo "/// Container ${CONTAINER} does not exist - run build_isaac to create it. Exiting cleanly. ///" >&2
    exit 0
fi

echo "/// Starting container ${CONTAINER}... ///"
# jtop.sock bind repair, required before EVERY start. /run is tmpfs, so the socket is gone
# after each boot until jtop.service recreates it. Starting the container while the bind
# source is missing makes Docker create it as a DIRECTORY, and every later `docker start`
# then fails with exit 127. run_dev.sh guards this at container CREATE only; `docker start`
# replays the stored bind list without re-checking it.
if docker container inspect "${CONTAINER}" \
        --format '{{range .HostConfig.Binds}}{{println .}}{{end}}' 2>/dev/null \
        | grep -q '^/run/jtop.sock:'; then

    # A non-socket here is that Docker-created directory; jtop cannot bind over it either.
    if [ -e /run/jtop.sock ] && [ ! -S /run/jtop.sock ]; then
        echo "/// /run/jtop.sock is not a socket - removing stale bind-mount artifact ///" >&2
        rm -rf /run/jtop.sock || true
    fi

    if [ ! -S /run/jtop.sock ]; then
        echo "/// jtop socket absent - starting jtop.service ///" >&2
        systemctl start --no-block jtop.service >/dev/null 2>&1 || true
        for _ in $(seq 1 15); do
            [ -S /run/jtop.sock ] && break
            sleep 1
        done
    fi

    # Starting now recreates the directory and re-enters that loop, so drop the mount
    # instead; run_dev.sh omits it when the socket is absent.
    if [ ! -S /run/jtop.sock ]; then
        echo "/// jtop socket unavailable - recreating ${CONTAINER} WITHOUT the jtop mount ///" >&2
        docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
        echo "/// Container removed. Run build_isaac to recreate it (run_dev.sh will skip jtop). ///" >&2
        exit 0
    fi
fi

docker start "${CONTAINER}"