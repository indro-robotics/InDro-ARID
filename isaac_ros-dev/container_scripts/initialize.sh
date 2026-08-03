#!/bin/bash
# initialize.sh - brings up the VSLAM stack through arid_supervisor.
# vslam_enable blocks until the supervisor has proven every configured RealSense up: allow
# ~3 min. Every camera protection (USB pre-check, node-up gate with a 40 s backstop, one
# reset_usb recovery) lives in the supervisor handler; this script has none of its own and
# only relays the response message.
# An already-running vslam answers "vslam already running" - that response is the whole of
# this script's idempotency.
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
    # Anchored on the CLI response-object line ('...Response(success=True, ...') so free
    # text inside message='...' can never spoof success.
    if echo "${out}" | grep -q 'Response(success=True'; then
        echo "  ${svc}: ok${msg:+ - ${msg}}"
        return 0
    fi
    echo "  ${svc}: refused${msg:+ - ${msg}}"
    return 1
}

# ros2 service list is a one-shot discovery snapshot: a cold CLI misses a service that is
# genuinely on the graph, so poll.
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

# Partial state (vslam half-started, USB mid-re-enumeration) is what makes a retry-in-place
# fail, so every retry tears the stack down first. The dominant bringup failure is a
# transient USB claim race (RS2_USB_STATUS_BUSY) that clears on a fresh attempt.
teardown() {
    echo "  tearing down before retry..."
    # 310 s: the disable queues behind an in-flight ~300 s enable; a shorter bound times
    # out doing nothing and the enable finishes into a running stack.
    call /arid_supervisor/vslam_enable false 310 >/dev/null 2>&1 || true
    sleep "${TEARDOWN_SETTLE_S:-15}"
}
BRINGUP_MAX_ATTEMPTS="${BRINGUP_MAX_ATTEMPTS:-3}"

# Ctrl+C or SSH death during the ~300 s vslam_enable strands the same partial stack, so
# signals tear down too.
on_signal() {
    local code="${1:-130}"
    # Ignore INT/TERM/PIPE BEFORE any write: escalating killers must not cut the teardown
    # short, and the reaped docker client makes writes raise SIGPIPE.
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
