#!/bin/bash
# Smoke test for host-side local_ws stack: systemd services, aliases,
# foxglove_bridge socket, cam_down lifecycle, rslidar lifecycle.
# Re-runnable. Leaves both pipelines STOPPED. Destructive aliases
# (reset_usb, clean_local) are existence-checked only, never invoked.

set -u

# ───────────────── output helpers ─────────────────
if [[ -t 1 ]]; then
    BLUE='\033[1;34m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
else
    BLUE=''; GREEN=''; YELLOW=''; RED=''; CYAN=''; BOLD=''; NC=''
fi

PASS=0
FAIL=0
SKIP=0
RESULTS=()

# Flags for resources started by this test (cleaned up in section 6 + EXIT trap).
# Each flag is cleared by the explicit cleanup once it confirms the resource is gone;
# anything still set when the EXIT trap fires is therefore a leak from an aborted run.
BRIDGE_LAUNCHED_BY_US=""
BRIDGE_LAUNCH_PID=""
CAM_DOWN_STARTED_BY_US=""
RSLIDAR_STARTED_BY_US=""

# Last-resort cleanup on any exit (Ctrl-C, set -e abort, normal end). Silent by design.
_cleanup_on_exit() {
    if [[ -n "${BRIDGE_LAUNCHED_BY_US}" && -n "${BRIDGE_LAUNCH_PID}" ]]; then
        kill -TERM -"${BRIDGE_LAUNCH_PID}" 2>/dev/null || true
    fi
    if [[ -n "${CAM_DOWN_STARTED_BY_US}" ]]; then
        bash -ic 'cam_down_stop' >/dev/null 2>&1 || true
    fi
    if [[ -n "${RSLIDAR_STARTED_BY_US}" ]]; then
        bash -ic 'rslidar_stop' >/dev/null 2>&1 || true
    fi
}
trap _cleanup_on_exit EXIT

hdr()  { echo -e "\n${BLUE}${BOLD}================================================================================${NC}"; echo -e "${BLUE}${BOLD}$*${NC}"; echo -e "${BLUE}${BOLD}================================================================================${NC}"; }
step() { echo -e "\n${BLUE}${BOLD}── $* ──${NC}"; }
what() { echo -e "${CYAN}WHAT:${NC}    $*"; }
why()  { echo -e "${CYAN}WHY:${NC}     $*"; }
raw()  { echo -e "${CYAN}RAW:${NC}";  echo "$1" | sed 's/^/         /'; }
pass() { echo -e "${GREEN}RESULT:  [PASS]  $*${NC}"; PASS=$((PASS+1)); RESULTS+=("PASS  $*"); }
fail() { echo -e "${RED}RESULT:  [FAIL]  $*${NC}";   FAIL=$((FAIL+1)); RESULTS+=("FAIL  $*"); }
skip() { echo -e "${YELLOW}RESULT:  [SKIP]  $*${NC}"; SKIP=$((SKIP+1)); RESULTS+=("SKIP  $*"); }
fix()  { echo -e "${CYAN}FIX:${NC}     $*"; }
note() { echo -e "         $*"; }

# Source ROS for non-interactive invocations.
if [[ -z "${ROS_DISTRO:-}" ]]; then
    [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
    [[ -f /home/jetson/workspaces/local_ws/install/setup.bash ]] && \
        source /home/jetson/workspaces/local_ws/install/setup.bash
fi
export ROS_DOMAIN_ID=23

# ───────────────── helpers ─────────────────
# Interactive bash so ARID-block aliases expand.
ialias() { bash -ic "$*" 2>&1 | grep -v 'job control'; }

# Count BEST_EFFORT messages over a wall-time window.
count_msgs() {
    local topic="$1" win="$2"
    timeout "$win" ros2 topic echo --no-arr --qos-reliability best_effort "$topic" 2>/dev/null \
        | grep -c '^---$'
}

hdr "Local Smoke Test"
echo ""
echo "Host:               $(hostname)"
echo "User:               $(id -un)"
echo "ROS_DISTRO:         ${ROS_DISTRO:-(missing!)}"
echo "ROS_DOMAIN_ID:      ${ROS_DOMAIN_ID}"
echo "AMENT_PREFIX_PATH:  $(echo "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | grep local_ws/install | head -2 | tr '\n' ' ')"

# ─────────────────────────────────────────────────────────────────────────────
hdr "Section 0 — Host systemd services"
step "Required services should be active"
what     "Three services are required: arid_description, gst_camera_manager, rslidar_coordinator."
why      "arid_description publishes /robot_description + TF static. The two managers expose SetBool/Trigger services that everything else depends on."
SVC_OUT=$(systemctl is-active arid_description.service gst_camera_manager.service rslidar_coordinator.service 2>&1)
raw "$(paste <(echo -e 'arid_description\ngst_camera_manager\nrslidar_coordinator') <(echo "${SVC_OUT}"))"

for svc in arid_description.service gst_camera_manager.service rslidar_coordinator.service; do
    if systemctl is-active --quiet "$svc"; then
        pass "$svc is active"
    else
        fail "$svc is NOT active"
        fix  "sudo systemctl restart $svc && journalctl -u $svc -n 50"
    fi
done

# ─────────────────────────────────────────────────────────────────────────────
hdr "Section 1 — Alias resolution"
step "All host-side aliases must be defined"
what     "Confirm each managed alias in the ARID block is loaded in an interactive shell."
why      "Aliases are how the operator drives the rig. A missing alias means setup.sh didn't run, or the bashrc block was overwritten."

EXPECTED_ALIASES=(
    reset_usb rosdep_local colcon_local clean_local
    foxglove_bridge
    cam_down_start cam_down_stop cam_down_status cam_down_alive
    rslidar_start rslidar_stop rslidar_status rslidar_alive rslidar_restart
    lidar_diag local_test
)
ALIAS_DUMP=$(ialias "alias")
for a in "${EXPECTED_ALIASES[@]}"; do
    LINE=$(echo "${ALIAS_DUMP}" | grep -E "^alias $a=" | head -1)
    if [[ -n "${LINE}" ]]; then
        pass "$a → $(echo "${LINE}" | sed -E "s/^alias $a='?(.*)'?$/\1/")"
    else
        fail "alias $a missing"
    fi
done

# ─────────────────────────────────────────────────────────────────────────────
hdr "Section 2 — Non-invasive checks for build/dep aliases"
step "Verify each destructive-ish alias points at a real, runnable target"
what     "reset_usb / rosdep_local / colcon_local / clean_local are not invoked here. We only check the things they would call/operate on actually exist."
why      "These four aliases are slow or destructive (clean_local wipes build/install/log). A smoke test shouldn't trigger them, but we still want to know they're not broken."

if [[ -x /home/jetson/workspaces/scripts/usb_reset.sh ]]; then
    pass "reset_usb target /home/jetson/workspaces/scripts/usb_reset.sh exists + executable"
else
    fail "reset_usb target missing or not executable"
fi
if [[ -d /home/jetson/workspaces/local_ws/src ]]; then
    pass "rosdep_local target /home/jetson/workspaces/local_ws/src/ exists"
else
    fail "rosdep_local target dir missing"
fi
if command -v colcon >/dev/null 2>&1; then
    pass "colcon_local: colcon at $(which colcon)"
else
    fail "colcon not on PATH"
fi
if colcon clean --help >/dev/null 2>&1; then
    pass "clean_local: colcon clean plugin available"
else
    fail "colcon clean plugin missing (apt install python3-colcon-clean)"
fi

# ─────────────────────────────────────────────────────────────────────────────
hdr "Section 3 — foxglove_bridge"
step "foxglove_bridge must be listening on TCP 8765"
what     "If no process is bound to 8765, launch the bridge via the alias and wait up to 5 s for the socket to open."
why      "Foxglove Studio connects via ws://<host>:8765. No socket = no Foxglove."

PORT_OUT=$(ss -tlnp 2>/dev/null | grep ':8765 ' || true)
if [[ -n "${PORT_OUT}" ]]; then
    raw "${PORT_OUT}"
    pass "port 8765 already listening (bridge was running)"
else
    note "no bridge running — launching via the foxglove_bridge alias (will be cleaned up in Section 6)"
    # setsid puts the launch in its own process group so Section 6 can killpg the whole tree.
    setsid bash -ic 'foxglove_bridge' >/tmp/foxglove_bridge.log 2>&1 < /dev/null &
    BRIDGE_LAUNCH_PID=$!
    BRIDGE_LAUNCHED_BY_US=1
    LAUNCHED=""
    for _ in 1 2 3 4 5; do
        sleep 1
        if ss -tlnp 2>/dev/null | grep -q ':8765 '; then
            LAUNCHED=1; break
        fi
    done
    PORT_OUT=$(ss -tlnp 2>/dev/null | grep ':8765 ' || echo '(still not listening)')
    raw "${PORT_OUT}"
    if [[ -n "${LAUNCHED}" ]]; then
        pass "bridge launched, port 8765 listening"
    else
        fail "bridge did not come up on 8765 within 5 s — see /tmp/foxglove_bridge.log"
    fi
fi

# ─────────────────────────────────────────────────────────────────────────────
hdr "Section 4 — cam_down lifecycle (CSI IMX219, sensor-id=0)"

# 4a. clean stopped state
step "4a. Force initial STOPPED state"
what     "Call cam_down_stop unconditionally so the test starts from a known state."
why      "Lifecycle test is meaningless if we don't know what the starting state was."
STOP_OUT=$(ialias 'cam_down_stop')
raw "${STOP_OUT}"
sleep 1
STATUS=$(ialias 'cam_down_status')
raw "${STATUS}"
if echo "${STATUS}" | grep -q 'cam_down STOPPED'; then
    pass "cam_down is STOPPED (clean baseline)"
else
    fail "could not establish baseline STOPPED state"
fi

# 4b. start
step "4b. cam_down_start should spawn the gst_cam_node subprocess"
what     "SetBool(true) on /gst_camera_manager/cam_down. The manager forks gst_cam_node as a subprocess."
why      "The whole pipeline depends on this subprocess. If it doesn't spawn, nothing else matters."
START_OUT=$(ialias 'cam_down_start')
raw "${START_OUT}"
CAM_DOWN_STARTED_BY_US=1   # cleared at 4i once explicit stop confirms STOPPED
PID=$(echo "${START_OUT}" | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2)
if echo "${START_OUT}" | grep -q "success=True" && [[ -n "${PID}" ]]; then
    pass "subprocess spawned (pid=${PID})"
else
    fail "cam_down_start did not produce a running subprocess"
fi

# 4c. argus + pipeline negotiation
step "4c. Wait for Argus + GStreamer pipeline to negotiate"
what     "nvarguscamerasrc takes ~3 s to negotiate sensor mode and start streaming."
why      "Tests on the topics will fail prematurely without this grace period."
note     "sleeping 4 s..."
sleep 4

# 4d. status running
step "4d. cam_down_status should report RUNNING"
STATUS=$(ialias 'cam_down_status')
raw "${STATUS}"
if echo "${STATUS}" | grep -q 'RUNNING'; then
    pass "status reports RUNNING"
else
    fail "status not RUNNING after start"
fi

# 4e. topics exist
step "4e. Three topics should exist: image_raw, image_raw/compressed, camera_info"
what     "Listing topics on the ROS graph; checking the three gst_cam_node publishers."
why      "If publishers aren't created, the gst_cam_node binary crashed (most common: empty encoding string, bad calibration path)."
TOPICS=$(ros2 topic list 2>/dev/null)
note     "cam_down topics in the graph:"
echo "${TOPICS}" | grep '^/cam_down' | sed 's/^/         /'
for t in /cam_down/image_raw /cam_down/image_raw/compressed /cam_down/camera_info; do
    if echo "${TOPICS}" | grep -q "^$t$"; then
        pass "topic $t published"
    else
        fail "topic $t missing"
    fi
done

# 4f. frame_id
step "4f. /cam_down/image_raw header.frame_id must equal 'bottom_visual_link'"
what     "Subscribe with matching BEST_EFFORT QoS and echo a single header."
why      "This is the TF frame that downstream consumers transform from. Wrong frame_id silently breaks every camera→base_link TF lookup."
HDR_OUT=$(timeout 5 ros2 topic echo --once --qos-reliability best_effort /cam_down/image_raw --field header 2>&1)
raw "${HDR_OUT}"
if echo "${HDR_OUT}" | grep -q 'frame_id: bottom_visual_link'; then
    pass "frame_id == bottom_visual_link"
else
    fail "frame_id is wrong or echo did not return"
fi

# 4g. rate
step "4g. /cam_down/image_raw rate over 5 s"
what     "Count message separators ('---') from 'topic echo --no-arr' over a 5-second window."
why      "Pipeline is configured for 20 fps (delivered ~16 Hz). Accept >= 6 Hz (~40% of delivered) as pass. Below that there's something wrong upstream (Argus dropping, ISP backpressure, etc.)."
COUNT=$(count_msgs /cam_down/image_raw 5)
HZ=$(awk "BEGIN {printf \"%.1f\", $COUNT/5}")
raw "messages: ${COUNT}    over: 5 s    rate: ${HZ} Hz"
if [[ "${COUNT}" -ge 30 ]]; then
    pass "rate ${HZ} Hz (${COUNT} msgs / 5 s) — healthy"
elif [[ "${COUNT}" -ge 1 ]]; then
    fail "rate only ${HZ} Hz — below 6 Hz threshold"
    note "check journalctl for the per-pipeline log under share/gst_camera_manager/logs/cam_down/"
else
    fail "no image_raw messages received in 5 s"
fi

# 4h. /alive
step "4h. /gst_camera_manager/cam_down/alive should be latched 'true'"
what     "Read the latched Bool with QoS RELIABLE / TRANSIENT_LOCAL / depth 1. Up to 10 s for first-time discovery."
why      "This is the manager's published verdict on whether frames are actually flowing. If status says RUNNING but alive says false, watchdog is seeing a stall."
ALIVE_OUT=$(timeout 10 ros2 topic echo --once \
    --qos-reliability reliable --qos-durability transient_local --qos-depth 1 \
    /gst_camera_manager/cam_down/alive 2>&1)
raw "${ALIVE_OUT}"
ALIVE_VAL=$(echo "${ALIVE_OUT}" | grep -oE 'data: (true|false)' | head -1)
case "${ALIVE_VAL}" in
    "data: true")  pass "alive == true (watchdog confirms frame flow)" ;;
    "data: false") fail "alive == false (watchdog says stalled despite status RUNNING)" ;;
    *)             fail "alive topic unreadable in 10 s (DDS discovery problem?)" ;;
esac

# 4i. stop
step "4i. cam_down_stop should terminate cleanly"
STOP_OUT=$(ialias 'cam_down_stop')
raw "${STOP_OUT}"
sleep 1
STATUS=$(ialias 'cam_down_status')
raw "${STATUS}"
if echo "${STATUS}" | grep -q 'STOPPED'; then
    pass "cam_down stopped cleanly"
    CAM_DOWN_STARTED_BY_US=""   # explicit stop succeeded; EXIT trap no longer needed
else
    fail "cam_down did not stop"
fi

# ─────────────────────────────────────────────────────────────────────────────
hdr "Section 5 — rslidar lifecycle (RSAIRY)"
note "Note: cloud-data flow depends on whether the physical LiDAR is reachable."
note "      Software-side lifecycle is exercised regardless of hardware."

# 5a. baseline stopped
step "5a. Force initial STOPPED state"
ialias 'rslidar_stop' >/dev/null
sleep 1
STATUS=$(ialias 'rslidar_status')
raw "${STATUS}"
if echo "${STATUS}" | grep -q 'STOPPED'; then
    pass "rslidar is STOPPED (clean baseline)"
else
    fail "could not establish baseline STOPPED state"
fi

# 5b. start
step "5b. rslidar_start should spawn rslidar_sdk_node via the coordinator"
what     "SetBool(true) on /rslidar_coordinator/enable. Coordinator forks 'ros2 run rslidar_sdk rslidar_sdk_node ...' with our config_path param."
why      "If this fails, the SDK config is missing/wrong, or the rslidar_sdk package wasn't built."
START_OUT=$(ialias 'rslidar_start')
raw "${START_OUT}"
RSLIDAR_STARTED_BY_US=1   # cleared at 5h once explicit stop confirms STOPPED
PID=$(echo "${START_OUT}" | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2)
if echo "${START_OUT}" | grep -q "success=True" && [[ -n "${PID}" ]]; then
    pass "subprocess spawned (pid=${PID})"
else
    fail "rslidar_start did not produce a running subprocess"
fi
note     "(letting SDK initialize for 4 s...)"
sleep 4

# 5c. status
step "5c. rslidar_status should report RUNNING"
STATUS=$(ialias 'rslidar_status')
raw "${STATUS}"
if echo "${STATUS}" | grep -q 'RUNNING'; then
    pass "status reports RUNNING"
else
    fail "status not RUNNING"
fi

# 5d. topics
step "5d. Required rslidar topics should exist on the graph"
what     "/rslidar_points (PointCloud2 from SDK), /rslidar_coordinator/alive (our supervisor's health Bool)."
why      "Topic presence proves the SDK + coordinator publishers are up. Data flow is the next check."
TOPICS=$(ros2 topic list 2>/dev/null)
note     "rslidar-related topics in the graph:"
echo "${TOPICS}" | grep -E 'rslidar' | sed 's/^/         /'
for t in /rslidar_points /rslidar_coordinator/alive; do
    if echo "${TOPICS}" | grep -q "^$t$"; then
        pass "topic $t published"
    else
        fail "topic $t missing"
    fi
done

# 5e_alive. /rslidar_coordinator/alive read
step "5e. /rslidar_coordinator/alive (read latched Bool via rslidar_alive alias)"
what     "Read latched /rslidar_coordinator/alive — same QoS as cam_down_alive (RELIABLE/TRANSIENT_LOCAL/depth 1)."
why      "Mirrors the cam_down_alive check. With LiDAR off, watchdog will report 'data: false' after the 5s startup-grace window."
ALIVE_OUT=$(timeout 10 ros2 topic echo --once \
    --qos-reliability reliable --qos-durability transient_local --qos-depth 1 \
    /rslidar_coordinator/alive 2>&1)
raw "${ALIVE_OUT}"
ALIVE_VAL=$(echo "${ALIVE_OUT}" | grep -oE 'data: (true|false)' | head -1)
case "${ALIVE_VAL}" in
    "data: true")  pass "alive == true (watchdog confirms cloud flow — LiDAR is reachable!)" ;;
    "data: false") pass "alive == false (latched topic readable; LiDAR not flowing data, which matches reality)" ;;
    *)             fail "alive topic unreadable in 10 s (DDS discovery problem?)" ;;
esac

# 5f. cloud data flow (informational)
step "5f. Cloud-data flow on /rslidar_points (hardware-dependent)"
what     "Count messages over 6 s (RSAIRY nominal ~10 Hz)."
why      "Tests whether the LiDAR is physically reachable. SKIP (not FAIL) if zero — that's a hardware issue, not a software defect."
COUNT=$(count_msgs /rslidar_points 6)
HZ=$(awk "BEGIN {printf \"%.1f\", $COUNT/6}")
raw "messages: ${COUNT}    over: 6 s    rate: ${HZ} Hz"
if [[ "${COUNT}" -ge 1 ]]; then
    pass "cloud flowing at ~${HZ} Hz (${COUNT} msgs / 6 s) — LiDAR reachable"
else
    skip "no cloud messages — LiDAR likely powered off / unreachable. Run 'lidar_diag' to debug network/hardware."
fi

# 5g. restart
step "5g. rslidar_restart should produce a new PID"
what     "Trigger /rslidar_coordinator/restart, then verify status reports a new pid different from before."
why      "Restart is the 'kick' for when the SDK is wedged. If PID doesn't change, the restart didn't actually re-spawn."
OLD_PID="${PID:-0}"
RESTART_OUT=$(ialias 'rslidar_restart')
raw "${RESTART_OUT}"
sleep 3
STATUS=$(ialias 'rslidar_status')
raw "${STATUS}"
NEW_PID=$(echo "${STATUS}" | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2)
if echo "${STATUS}" | grep -q 'RUNNING' && [[ -n "${NEW_PID}" ]] && [[ "${NEW_PID}" != "${OLD_PID}" ]]; then
    pass "restart produced a new pid (${OLD_PID} → ${NEW_PID})"
else
    fail "restart did not produce a new pid (was ${OLD_PID}, now ${NEW_PID:-unknown})"
fi

# 5h. stop
step "5h. rslidar_stop should terminate cleanly (coordinator now waits for the whole process group)"
note     "(With the process-group fix in place, rslidar_stop only returns once the SDK binary is genuinely gone — even if it was busy in MSOPTIMEOUT retries.)"
ialias 'rslidar_stop' >/dev/null
STATUS=$(ialias 'rslidar_status')
raw "${STATUS}"
if echo "${STATUS}" | grep -q 'STOPPED'; then
    pass "rslidar stopped cleanly"
    RSLIDAR_STARTED_BY_US=""   # explicit stop succeeded; EXIT trap no longer needed
else
    fail "rslidar did not stop"
fi

# ─────────────────────────────────────────────────────────────────────────────
hdr "Section 6 — Final cleanup"

if [[ -n "${BRIDGE_LAUNCHED_BY_US}" ]]; then
    step "Stop foxglove_bridge launched by this test"
    what     "SIGTERM the bridge's process group (PGID ${BRIDGE_LAUNCH_PID}), fall back to SIGKILL, verify port 8765 freed."
    why      "Section 3 launched the bridge in the background. A test that leaves a bridge bound to 8765 prevents the next 'foxglove_bridge' invocation from binding the port."
    kill -TERM -"${BRIDGE_LAUNCH_PID}" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
        sleep 1
        ss -tlnp 2>/dev/null | grep -q ':8765 ' || break
    done
    if ss -tlnp 2>/dev/null | grep -q ':8765 '; then
        note "still listening after 5 s — escalating to SIGKILL"
        kill -KILL -"${BRIDGE_LAUNCH_PID}" 2>/dev/null || true
        sleep 1
    fi
    if ss -tlnp 2>/dev/null | grep -q ':8765 '; then
        fail "could not free port 8765 (foxglove_bridge still holding it)"
    else
        pass "bridge stopped, port 8765 free"
        BRIDGE_LAUNCHED_BY_US=""   # tell the EXIT trap there's nothing left to do
    fi
else
    step "foxglove_bridge"
    note "bridge was already running before this test; leaving it alone."
fi

step "Both pipelines should be stopped, no leaked subprocesses"
what     "After stop, no gst_cam_node or rslidar_sdk_node processes should remain."
why      "Process leaks indicate a bug in the subprocess-termination logic of either manager (signal not propagated to child, missing killpg, etc.)."
LEAK=$(pgrep -af 'gst_cam_node|rslidar_sdk_node' 2>/dev/null | grep -v 'bash -c\|pgrep' || true)
if [[ -z "${LEAK}" ]]; then
    pass "no managed subprocesses running"
else
    fail "the following processes are still alive:"
    echo "${LEAK}" | sed 's/^/         /'
fi

# ─────────────────────────────────────────────────────────────────────────────
hdr "Summary"
echo ""
echo -e "  ${GREEN}Passed:${NC}   ${PASS}"
echo -e "  ${RED}Failed:${NC}   ${FAIL}"
echo -e "  ${YELLOW}Skipped:${NC}  ${SKIP}"
if [[ "${FAIL}" -gt 0 ]]; then
    echo ""
    echo "  Failed items:"
    printf '%s\n' "${RESULTS[@]}" | grep '^FAIL' | sed 's/^/    /'
fi
if [[ "${SKIP}" -gt 0 ]]; then
    echo ""
    echo "  Skipped items:"
    printf '%s\n' "${RESULTS[@]}" | grep '^SKIP' | sed 's/^/    /'
fi
echo -e "${BOLD}================================================================================${NC}"
exit "${FAIL}"
