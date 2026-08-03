#!/bin/bash
# airborne_check.sh - exits 0 only on a fresh 'landed: false' sample.
# arid_supervisor.service ExecStopPost reaps the running stack on every non-zero exit, so a
# missing publisher, an absent sample and any error all exit 1: an ambiguous case fails
# toward cleanup, never toward leaving an unattended stack up.
# ARID_LAND_TOPIC retargets the check. A topic nothing publishes reads as not airborne and
# the reap then runs under a flying aircraft.
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
