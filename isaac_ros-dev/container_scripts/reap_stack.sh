#!/bin/bash
# Reaps the vslam launch tree: group-SIGINT, 25 s drain, group-SIGKILL. Run by
# arid_supervisor.service ExecStopPost, only after airborne_check.sh reports not airborne.
# A file rather than an inline ExecStopPost command because systemd expands $VAR itself:
# an unset variable turns into pkill -f "" and matches every process on the machine.
# This runs in a --pid=host container as root, so both patterns reach HOST processes. Any
# pattern that is not specific to the px4_vslam launch also kills the host ROS units
# (rslidar_coordinator, its rslidar_sdk child, gst_camera_manager, arid_description). Keep
# the match fully qualified and [.]-escaped, and in sync with the copy in
# arid_supervisor_node.py.
P='ros2 launch px4_vslam vslam[.]launch[.]py'
# An orphaned component container carries no 'ros2 launch' token: matching P alone reports
# nothing to reap while the container stays up holding the cameras and the ROS graph.
C='component_container.*__node:=vslam_container'
# Signal the matched pid's REAL pgid, never the pid: a launch started from a shell is not
# its own group leader, so -pid raises ESRCH and its children survive holding the cameras,
# and a pid equal to an unrelated pgid signals that group instead. The parents are setsid
# leaders, so the group covers children that outlive the launch parent.
pgids=""
for p in $(pgrep -f "${P}"; pgrep -f "${C}"); do
    # Excluded so this script's own pgid never enters the kill set: it must survive to run
    # the drain and the SIGKILL escalation.
    [ "${p}" = "$$" ] && continue
    [ "${p}" = "${PPID}" ] && continue
    g=$(ps -o pgid= -p "${p}" 2>/dev/null | tr -d ' ')
    [ -n "${g}" ] || continue
    case " ${pgids} " in *" ${g} "*) ;; *) pgids="${pgids} ${g}" ;; esac
done
[ -n "${pgids}" ] || { echo "reap_stack: nothing to reap"; exit 0; }
echo "reap_stack: reaping groups:" ${pgids}
for g in ${pgids}; do
    kill -INT -- "-${g}" 2>/dev/null
done
for i in $(seq 1 25); do
    alive=0
    for g in ${pgids}; do
        pgrep -g "${g}" >/dev/null 2>&1 && { alive=1; break; }
    done
    [ "${alive}" = 0 ] && { echo "reap_stack: drained in ${i}s"; exit 0; }
    sleep 1
done
for g in ${pgids}; do
    kill -KILL -- "-${g}" 2>/dev/null
done
echo "reap_stack: SIGKILL escalation after 25s"
exit 0
