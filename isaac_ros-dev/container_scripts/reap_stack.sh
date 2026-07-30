#!/bin/bash
# reap_stack.sh - group-SIGINT -> 25 s drain -> group-SIGKILL the vslam launch tree. Run by
# ExecStopPost after airborne_check.sh clears it. Script, not inline: systemd expands $VAR
# itself (unset -> pkill -f "" = match everything).
# The pattern is 'ros2 launch <pkg> <file>'-qualified + [.]-escaped so it can never hit the
# host-side gst_camera_manager / arid_description units (shared PID namespace). Kills whole
# process GROUPS (the launch parent is a setsid leader): a wedged child outliving its parent
# must not survive holding a camera; the drain checks group members, not just parents.
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
