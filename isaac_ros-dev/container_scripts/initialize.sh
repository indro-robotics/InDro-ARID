#!/bin/bash
# initialize.sh - start the VSLAM stack via the supervisor:
#   /arid_supervisor/vslam_enable -> true   (BLOCKING + CAMERA-PROVEN: the supervisor runs
#                                            the USB pre-check, the RealSense up-gate bounded
#                                            by a 40 s backstop, and ONE reset_usb recovery.
#                                            Allow up to ~3 min.)
# Idempotent: on top of an already-running vslam the supervisor answers "vslam already
# running (...)" and this exits ok. The supervisor is started at boot by
# arid_supervisor.service on the host.
set -u

call() {
    # call <service> <true|false> [timeout_s] - default 30 s; the camera-gated
    # vslam_enable is the one legitimately long call and passes its own bound.
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
    # Anchored on the CLI response-object line so free text inside message='...' cannot
    # spoof success.
    if echo "${out}" | grep -q 'Response(success=True'; then
        echo "  ${svc}: ok${msg:+ - ${msg}}"
        return 0
    fi
    echo "  ${svc}: refused${msg:+ - ${msg}}"
    return 1
}

# `ros2 service list` is a one-shot discovery snapshot, so poll it: a cold CLI can miss a
# service that is genuinely on the graph.
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

# A failed enable can leave vslam half-started with USB mid-re-enumeration; retry from
# clean rather than in place. The dominant bringup failure is a transient USB claim race
# (RS2_USB_STATUS_BUSY) that clears on a fresh attempt.
teardown() {
    echo "  tearing down before retry..."
    # 310 s: the disable queues behind an in-flight ~300 s enable; a shorter bound times
    # out doing nothing and the enable finishes into a running stack.
    call /arid_supervisor/vslam_enable false 310 >/dev/null 2>&1 || true
    sleep "${TEARDOWN_SETTLE_S:-15}"
}
BRINGUP_MAX_ATTEMPTS="${BRINGUP_MAX_ATTEMPTS:-3}"

# Ctrl+C / SSH death during the ~300 s camera-gated enable would otherwise strand vslam
# mid-USB-re-enumeration - exactly the partial state that makes a retry-in-place fail.
on_signal() {
    local code="${1:-130}"
    # Ignore INT/TERM/PIPE BEFORE any write: escalating killers must not cut the teardown
    # short, and a reaped docker client makes writes raise SIGPIPE.
    trap "" INT TERM PIPE
    echo "  interrupted - tearing the partial stack down" >&2 || true
    teardown
    exit "${code}"
}
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

for BRINGUP_ATTEMPT in $(seq 1 "${BRINGUP_MAX_ATTEMPTS}"); do
    # vslam_enable BLOCKS until the supervisor has PROVEN the cameras up (or failed after
    # its single reset_usb recovery). call() prints the response message - the success
    # timing or the verbatim failure evidence. Full detail: supervisor journal +
    # ${ISAAC_ROS_WS}/run_logs/vslam/vslam.log.
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
