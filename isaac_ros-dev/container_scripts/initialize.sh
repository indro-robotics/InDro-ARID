#!/bin/bash
# Start the VSLAM stack via the supervisor:
#   /vslam_supervisor/vslam_enable -> true
# Idempotent. The supervisor itself is started at boot by vslam_supervisor.service.
set -u

call() {
    local svc="$1" data="$2" out msg
    if ! out=$(ros2 service call "${svc}" std_srvs/srv/SetBool "{data: ${data}}" 2>&1); then
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

if ! ros2 service list 2>/dev/null | grep -q '^/vslam_supervisor/vslam_enable$'; then
    echo "ERROR: /vslam_supervisor/* services not on the graph - supervisor not running." >&2
    echo "       Check 'sudo systemctl status vslam_supervisor.service' on the host." >&2
    exit 1
fi

echo "Starting VSLAM stack:"
call /vslam_supervisor/vslam_enable true || exit 1
echo "Services come up in a few seconds."
