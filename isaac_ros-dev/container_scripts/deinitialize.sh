#!/bin/bash
# deinitialize.sh - stops the VSLAM stack through arid_supervisor.
# vslam_enable false carries the RealSense teardown and the shm sweep.
# The supervisor refuses the disable unless the drone is provably landed. The status probe
# below applies the same gate first so a refusal names its reason instead of consuming the
# 300 s call bound.
set -u

# Sized for the queue, not for the work: a stack stop is bounded inside the supervisor but
# queues behind an in-flight camera-gated bringup, worst ~3 min.
CALL_TIMEOUT_S=300
STATUS_TIMEOUT_S=300

call() {
    local svc="$1" data="$2" out msg rc
    # -k: teardown children inherit SIG_IGN, a plain timeout then blocks forever.
    out=$(timeout -k 10 "${CALL_TIMEOUT_S}" ros2 service call "${svc}" std_srvs/srv/SetBool "{data: ${data}}" 2>&1)
    rc=$?
    if [ "${rc}" -ne 0 ]; then
        [ "${rc}" -eq 124 ] && out="${out}
(no response within ${CALL_TIMEOUT_S}s - supervisor may be stalled)"
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

# ros2 service list is a one-shot discovery snapshot from a cold CLI, so poll it.
svc_up() {
    local svc="$1" tries="${2:-10}"
    for _ in $(seq 1 "${tries}"); do
        ros2 service list 2>/dev/null | grep -qx "${svc}" && return 0
        sleep 0.5
    done
    return 1
}

if ! svc_up /arid_supervisor/vslam_enable; then
    echo "ERROR: /arid_supervisor/* services not on the graph - supervisor not running." >&2
    echo "       Check 'sudo systemctl status arid_supervisor.service' on the host." >&2
    exit 1
fi

if svc_up /arid_supervisor/status; then
    st=$(timeout -k 10 "${STATUS_TIMEOUT_S}" ros2 service call /arid_supervisor/status std_srvs/srv/Trigger "{}" 2>&1)
    strc=$?
    if [ "${strc}" -ne 0 ]; then
        echo "ERROR: supervisor status probe failed/timed out - cannot verify the landed interlock; refusing teardown." >&2
        echo "${st}" | tail -3 | sed 's/^/    /' >&2
        exit 1
    fi
    stmsg=$(echo "${st}" | sed -n "s/.*message=['\"]\(.*\)['\"].*/\1/p" | head -1)
    land=$(echo "${stmsg}" | grep -o 'land: [a-z]*' | awk '{print $2}')
    if echo "${stmsg}" | grep -q 'running' && [ "${land}" != "landed" ]; then
        echo "ERROR: refused - vslam is running and the drone is not provably landed (${stmsg})." >&2
        echo "       Land first; the supervisor enforces this same gate." >&2
        exit 1
    fi
    echo "  interlock: ${stmsg}"
else
    echo "WARN: /arid_supervisor/status absent - proceeding without the pre-check (the" >&2
    echo "      supervisor still enforces its own land gate on the disable)." >&2
fi

echo "Stopping VSLAM stack:"
call /arid_supervisor/vslam_enable false || exit 1
