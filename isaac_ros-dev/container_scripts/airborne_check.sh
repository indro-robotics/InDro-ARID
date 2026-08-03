#!/bin/bash
# Airborne interlock for arid_supervisor.service ExecStopPost: exit 0 preserves the vslam
# tree, exit 1 lets reap_stack.sh kill it. Only a fresh 'landed: false' sample earns exit 0;
# every unprovable case exits 1, so an orphaned tree never survives holding the cameras.
TOPIC="${ARID_LAND_TOPIC:-/fmu/out/vehicle_land_detected}"
info=$(timeout -k 5 8 ros2 topic info --no-daemon "${TOPIC}" 2>/dev/null)
if ! grep -q 'Publisher count: [1-9]' <<<"${info}"; then
    echo "airborne_check: no publisher on ${TOPIC} - not airborne"
    exit 1
fi
out=""
for i in 1 2; do
    out=$(timeout -k 5 12 ros2 topic echo --once --no-daemon \
        --qos-reliability best_effort --qos-durability volatile \
        "${TOPIC}" px4_msgs/msg/VehicleLandDetected 2>/dev/null) && break
    out=""
done
if [ -z "${out}" ]; then
    echo "airborne_check: publisher present but no sample - treating as not airborne"
    exit 1
fi
if grep -q '^landed: false' <<<"${out}"; then
    echo "airborne_check: AIRBORNE - stack preserved"
    exit 0
fi
echo "airborne_check: landed"
exit 1
