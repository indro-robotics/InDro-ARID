#!/bin/bash
# Starts the VSLAM stack by enabling it on the supervisor, behind the `initialize` alias.
# The enable call blocks while the supervisor proves the cameras up: allow ~3 min per
# attempt. Idempotent: enabling an already-running stack succeeds without restarting it.
set -u

call() {
    local svc="$1" data="$2" tmo="${3:-30}" out msg rc
    # -k: teardown children inherit SIG_IGN, a plain timeout then blocks forever.
    out=$(timeout -k 10 "${tmo}" ros2 service call "${svc}" std_srvs/srv/SetBool "{data: ${data}}" 2>&1)
    rc=$?
    if [ "${rc}" -ne 0 ]; then
        [ "${rc}" -eq 124 ] && out="${out}
(no response within ${tmo}s - supervisor may be stalled)"
        echo "  ${svc}: call failed"
        echo "${out}" | sed 's/^/    /'
        return 1
    fi
    msg=$(echo "${out}" | sed -n "s/.*message=['\"]\(.*\)['\"].*/\1/p" | head -1)
    # Anchored on the CLI response-object line: free text inside message='...' would
    # otherwise spoof success.
    if echo "${out}" | grep -q 'Response(success=True'; then
        echo "  ${svc}: ok${msg:+ - ${msg}}"
        return 0
    fi
    echo "  ${svc}: refused${msg:+ - ${msg}}"
    return 1
}

# `ros2 service list` is a one-shot discovery snapshot: a cold CLI misses services that are
# genuinely on the graph, so poll instead of asking once.
SERVICE_WAIT_S="${SERVICE_WAIT_S:-15}"
service_up() {
    local svc="$1" end=$(( SECONDS + ${2:-${SERVICE_WAIT_S}} ))
    while [ "${SECONDS}" -lt "${end}" ]; do
        ros2 service list 2>/dev/null | grep -qx "${svc}" && return 0
        sleep 0.5
    done
    return 1
}

if ! service_up /arid_supervisor/vslam_enable; then
    echo "ERROR: /arid_supervisor/* services not on the graph - supervisor not running." >&2
    echo "       Check 'sudo systemctl status arid_supervisor.service' on the host." >&2
    exit 1
fi

echo "Starting VSLAM stack:"

# A failed enable leaves vslam half-started with USB mid-re-enumeration, and a retry in
# place fails on that state, so every retry runs from a full teardown. The dominant failure
# is a transient librealsense USB claim race (RS2_USB_STATUS_BUSY) that clears on a retry.
teardown() {
    echo "  tearing down before retry..."
    # 310 s: the disable queues behind an in-flight ~300 s enable. A shorter bound times out
    # having done nothing, and the enable then completes into a stack believed torn down.
    call /arid_supervisor/vslam_enable false 310 >/dev/null 2>&1 || true
    sleep "${TEARDOWN_SETTLE_S:-15}"
}
BRINGUP_MAX_ATTEMPTS="${BRINGUP_MAX_ATTEMPTS:-3}"

# Without this trap, Ctrl+C or a dropped SSH session during the ~300 s enable strands vslam
# mid-USB-re-enumeration.
on_signal() {
    local code="${1:-130}"
    # Ignore INT/TERM/PIPE BEFORE any write: a second signal must not cut the teardown short,
    # and writes raise SIGPIPE once the docker client is reaped.
    trap "" INT TERM PIPE
    echo "  interrupted - tearing the partial stack down" >&2 || true
    teardown
    exit "${code}"
}
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

for BRINGUP_ATTEMPT in $(seq 1 "${BRINGUP_MAX_ATTEMPTS}"); do
    if call /arid_supervisor/vslam_enable true 300; then
        [ "${BRINGUP_ATTEMPT}" -gt 1 ] && echo "Bringup succeeded on attempt ${BRINGUP_ATTEMPT}/${BRINGUP_MAX_ATTEMPTS}."
        echo "Services come up in a few seconds."
        exit 0
    fi
    echo "  vslam_enable refused (attempt ${BRINGUP_ATTEMPT}/${BRINGUP_MAX_ATTEMPTS})" >&2
    [ "${BRINGUP_ATTEMPT}" -lt "${BRINGUP_MAX_ATTEMPTS}" ] && teardown
done

teardown
echo "ERROR: bringup failed ${BRINGUP_MAX_ATTEMPTS}/${BRINGUP_MAX_ATTEMPTS} attempts; stack torn down." >&2
exit 1
