#!/bin/bash
set -e

CONTAINER=isaac_ros_dev-aarch64-container

# With no container record `docker start` exits non-zero and systemd loops
# Restart=on-failure forever; exiting 0 instead lands the unit in "active (exited)".
if ! docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
    echo "/// Container ${CONTAINER} does not exist - run build_isaac to create it. Exiting cleanly. ///" >&2
    exit 0
fi

echo "/// Starting container ${CONTAINER}... ///"
# /run is tmpfs, so /run/jtop.sock is gone every boot until jtop.service recreates it.
# Starting the container first makes Docker create the missing bind source as a DIRECTORY,
# after which every `docker start` fails with exit 127 and systemd loops forever. run_dev.sh
# guards this only at container CREATE: `docker start` replays the stored bind list unchecked.
if docker container inspect "${CONTAINER}" \
        --format '{{range .HostConfig.Binds}}{{println .}}{{end}}' 2>/dev/null \
        | grep -q '^/run/jtop.sock:'; then

    # A non-socket here is the Docker-created directory; jtop cannot bind over it either.
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

    # Starting with the socket still absent recreates the directory and re-enters the loop,
    # so drop the mount by recreating the container: run_dev.sh then omits it.
    if [ ! -S /run/jtop.sock ]; then
        echo "/// jtop socket unavailable - recreating ${CONTAINER} WITHOUT the jtop mount ///" >&2
        docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
        echo "/// Container removed. Run build_isaac to recreate it (run_dev.sh will skip jtop). ///" >&2
        exit 0
    fi
fi

docker start "${CONTAINER}"