#!/bin/bash
# reap_stack.sh - group-SIGINT, 25 s drain, group-SIGKILL of the vslam launch tree.
# Run by arid_supervisor.service ExecStopPost once airborne_check.sh reports not airborne.
# A file and not an inline ExecStopPost: systemd expands $VAR itself, and one unset variable
# leaves `pkill -f ""`, which matches every process on the host.
# The pattern stays 'ros2 launch <pkg> <file>'-qualified and [.]-escaped so it cannot reach
# the host-side gst_camera_manager or arid_description units through the shared PID namespace.
P='ros2 launch px4_vslam vslam[.]launch[.]py'
# Group-kill by the matched pid's REAL pgid, never by the pid itself: a launch started from
# a shell is not its own group leader, so -pid raises ESRCH and a single kill leaves the
# container children holding the cameras; and a pid that equals an unrelated group's pgid
# signals that innocent group. The drain below checks group members, not only parents.
# An orphaned component container carries no 'ros2 launch' token, so matching the launch
# parents alone reports "nothing to reap" while the container still holds the cameras and
# the ROS graph.
C='component_container.*__node:=vslam_container'
pgids=""
for p in $(pgrep -f "${P}"; pgrep -f "${C}"); do
    # never signal our own group: this script and its shell must survive the reap
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
