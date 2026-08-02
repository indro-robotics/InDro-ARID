#!/bin/bash
set -e

CONTAINER=isaac_ros_dev-aarch64-container

# A missing container record makes `docker start` fail and systemd restart-loop indefinitely;
# probe first and exit cleanly.
if ! docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
    echo "/// Container ${CONTAINER} does not exist - run build_isaac to create it. Exiting cleanly. ///" >&2
    exit 0
fi

echo "/// Starting container ${CONTAINER}... ///"
# jtop.sock bind repair. /run is tmpfs, so the socket dies every boot and only jtop.service
# recreates it. Start the container first and Docker auto-creates the missing bind source as a
# DIRECTORY, after which every `docker start` fails with exit 127 and systemd loops forever.
# run_dev.sh guards this, but only at container CREATE - `docker start` replays the stored bind
# list and never re-checks. Repair the source here, before every start.
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

    # Still no socket: starting now recreates the directory and re-enters the loop. Drop the
    # mount instead by recreating the container - run_dev.sh then omits it.
    if [ ! -S /run/jtop.sock ]; then
        echo "/// jtop socket unavailable - recreating ${CONTAINER} WITHOUT the jtop mount ///" >&2
        docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
        echo "/// Container removed. Run build_isaac to recreate it (run_dev.sh will skip jtop). ///" >&2
        exit 0
    fi
fi

docker start "${CONTAINER}"