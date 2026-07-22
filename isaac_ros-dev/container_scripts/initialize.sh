#!/bin/bash
# Start the VSLAM stack via the supervisor:
#   /arid_supervisor/vslam_enable -> true
# Idempotent. The supervisor itself is started at boot by arid_supervisor.service.
set -u

# The camera-proven bringup gate blocks the enable call ~15 s (healthy) up to ~3 min
# (double reset_usb recovery), so cap it generously and surface a wedged supervisor.
CALL_TIMEOUT_S=300

call() {
    local svc="$1" data="$2" out rc msg
    out=$(timeout "${CALL_TIMEOUT_S}" ros2 service call "${svc}" std_srvs/srv/SetBool "{data: ${data}}" 2>&1)
    rc=$?
    if [ "${rc}" -eq 124 ]; then
        echo "  ${svc}: no response within ${CALL_TIMEOUT_S}s (supervisor may be wedged)"
        return 1
    fi
    if [ "${rc}" -ne 0 ]; then
        echo "  ${svc}: call failed"
        echo "${out}" | sed 's/^/    /'
        return 1
    fi
    msg=$(echo "${out}" | sed -n "s/.*message=['\"]\(.*\)['\"].*/\1/p" | head -1)
    if echo "${out}" | grep -q 'success=True'; then
        echo "  ${svc}: ok${msg:+ - ${msg}}"
        return 0
    fi
    echo "  ${svc}: refused${msg:+ - ${msg}}"
    return 1
}

if ! ros2 service list 2>/dev/null | grep -q '^/arid_supervisor/vslam_enable$'; then
    echo "ERROR: /arid_supervisor/* services not on the graph - supervisor not running." >&2
    echo "       Check 'sudo systemctl status arid_supervisor.service' on the host." >&2
    exit 1
fi

echo "Starting VSLAM stack:"
call /arid_supervisor/vslam_enable true || exit 1
echo "Services come up in a few seconds."
