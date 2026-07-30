#!/bin/bash
# reap_stack.sh - group-SIGINT -> 25 s drain -> group-SIGKILL the vslam launch tree.
# Run by ExecStopPost after airborne_check.sh clears it. Script, not inline: systemd
# expands $VAR itself (unset -> pkill -f "" = match everything).
# The pattern is fully qualified 'ros2 launch <pkg> <file>' and [.]-escaped. This runs in a
# --pid=host container as root, so it can see and signal HOST processes: the host units
# rslidar_coordinator (ros2 launch rslidar_coordinator ...), its rslidar_sdk child
# (ros2 run rslidar_sdk ...), gst_camera_manager and arid_description must all survive, and
# none of them carries the px4_vslam token.
# Kills whole process GROUPS (the parents are setsid leaders): a wedged child outliving its
# launch parent must not survive holding the camera; the drain checks group members, not
# just parents.
P='ros2 launch px4_vslam vslam[.]launch[.]py'
# Group-kill by the matched pid's REAL pgid, never by the pid itself: a launch started
# from a shell (dev alias) is not its own group leader, so -pid would ESRCH and the
# fallback single kill leaves the container children holding the cameras - and a pid
# that happens to equal an unrelated group's pgid would signal that innocent group.
# Orphan-safe discovery: an orphaned component container carries no 'ros2 launch' token,
# so matching parents alone reports "nothing to reap" while the container is still up
# holding the cameras and the ROS graph (proven live: a killed launch parent left
# component_container_mt running and this script reported nothing to do). Match the
# containers by their own node name too; both sets reduce to pgids below. The node
# names belong to the launches in P above, so this cannot widen the blast radius.
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
