#!/bin/bash
# Diagnostic for the RSAIRY LiDAR's host-side network + SDK state.
# Read-only. Sudo needed for steps 5 and 6 (tcpdump, arp-scan).

set -u

NIC="enP8p1s0"
EXPECTED_LIDAR_IP="192.168.1.200"
HOST_IP="192.168.1.102"
SUBNET="192.168.1.0/24"
EXPECTED_LIDAR_MAC=""
EXPECTED_MSOP=6699
EXPECTED_DIFOP=7788
EXPECTED_IMU=6688

# Auto-detected values override the factory defaults above.
WORKSPACES="${WORKSPACES:-/home/jetson/workspaces}"
DETECTED_CONF="${WORKSPACES}/.lidar/rslidar_detected.conf"
DETECTED=""
if [[ -r "${DETECTED_CONF}" ]]; then
    # shellcheck disable=SC1090
    source "${DETECTED_CONF}"
    NIC="${RSLIDAR_NIC:-${NIC}}"
    EXPECTED_LIDAR_IP="${RSLIDAR_LIDAR_IP:-${EXPECTED_LIDAR_IP}}"
    HOST_IP="${RSLIDAR_HOST_IP:-${HOST_IP}}"
    SUBNET="${RSLIDAR_SUBNET:-${SUBNET}}"
    EXPECTED_LIDAR_MAC="${RSLIDAR_LIDAR_MAC:-}"
    DETECTED="yes"
fi

# Output helpers
if [[ -t 1 ]]; then
    BLUE='\033[1;34m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
else
    BLUE=''; GREEN=''; YELLOW=''; RED=''; CYAN=''; BOLD=''; NC=''
fi
hdr()      { echo -e "\n${BLUE}${BOLD}================================================================================${NC}"; echo -e "${BLUE}${BOLD}$*${NC}"; echo -e "${BLUE}${BOLD}================================================================================${NC}"; }
step()     { echo -e "\n${BLUE}${BOLD}-- $* --${NC}"; }
what()     { echo -e "${CYAN}WHAT:${NC}    $*"; }
why()      { echo -e "${CYAN}WHY:${NC}     $*"; }
raw()      { echo -e "${CYAN}RAW:${NC}"; echo "$1" | sed 's/^/         /'; }
ok()       { echo -e "${GREEN}RESULT:  [OK]    $*${NC}"; }
warn()     { echo -e "${YELLOW}RESULT:  [WARN]  $*${NC}"; }
fail()     { echo -e "${RED}RESULT:  [FAIL]  $*${NC}"; }
fix()      { echo -e "${CYAN}FIX:${NC}     $*"; }
info()     { echo -e "${CYAN}INFO:${NC}    $*"; }
note()     { echo -e "         $*"; }

LIDAR_FOUND=""
LIDAR_SRC_IP=""
LIDAR_DST_IP=""
LIDAR_DST_PORT=""
SUDO_OK="no"
sudo -n true 2>/dev/null && SUDO_OK="yes"

hdr "RSAIRY LiDAR Network Diagnostic"
echo ""
echo "Host:           $(hostname)"
echo "User:           $(id -un)"
echo "Date:           $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "NIC under test: ${NIC}"
if [[ -n "${DETECTED}" ]]; then
    echo "Source of IPs:  ${DETECTED_CONF}  (auto-detected)"
    echo "Expected:       LiDAR ${EXPECTED_LIDAR_IP}  (MAC ${EXPECTED_LIDAR_MAC:-?})"
    echo "                Host  ${HOST_IP}            on subnet ${SUBNET}"
else
    echo "Source of IPs:  factory defaults  (no ${DETECTED_CONF})"
    echo "Expected:       LiDAR ${EXPECTED_LIDAR_IP},  Host ${HOST_IP}/24"
    echo "                Run 'sudo config_lidar' to auto-detect the real IPs."
fi
echo "UDP ports:      MSOP/${EXPECTED_MSOP}, DIFOP/${EXPECTED_DIFOP}, IMU/${EXPECTED_IMU}"
echo "Sudo cached:    ${SUDO_OK}  $( [[ "$SUDO_OK" == "no" ]] && echo "(steps 5 & 6 will be skipped)" )"

step "1. Interface presence + electrical link"
what     "Is ${NIC} present, and does it have a live cable (carrier)?"
why      "No carrier means cable unplugged, peer offline, or broken cable."

if ! ip link show "${NIC}" >/dev/null 2>&1; then
    fail "interface ${NIC} not present"
    fix  "ip -o link show  # to find the actual NIC name"
    exit 1
fi
LINK_LINE=$(ip -o link show "${NIC}")
raw "${LINK_LINE}"

if echo "${LINK_LINE}" | grep -qE 'LOWER_UP'; then
    SPEED=$(ethtool "${NIC}" 2>/dev/null | awk -F: '/Speed:/ {gsub(/^[ \t]+/, "", $2); print $2}')
    DUPLEX=$(ethtool "${NIC}" 2>/dev/null | awk -F: '/Duplex:/ {gsub(/^[ \t]+/, "", $2); print $2}')
    MTU=$(echo "${LINK_LINE}" | awk '{for(i=1;i<=NF;i++) if($i=="mtu") print $(i+1)}')
    MAC=$(echo "${LINK_LINE}" | grep -oE 'link/ether [0-9a-f:]+' | awk '{print $2}')
    RX_BYTES=$(grep -E "^\s*${NIC}:" /proc/net/dev | awk -F: '{print $2}' | awk '{print $1}')
    RX_PKTS=$(grep -E "^\s*${NIC}:" /proc/net/dev | awk -F: '{print $2}' | awk '{print $2}')
    RX_ERRS=$(grep -E "^\s*${NIC}:" /proc/net/dev | awk -F: '{print $2}' | awk '{print $3}')
    RX_DROP=$(grep -E "^\s*${NIC}:" /proc/net/dev | awk -F: '{print $2}' | awk '{print $4}')
    ok "carrier UP"
    note "speed:       ${SPEED:-?} ${DUPLEX:-?}"
    note "MTU:         ${MTU:-?}"
    note "host MAC:    ${MAC:-?}"
    note "RX lifetime: ${RX_PKTS} packets, ${RX_BYTES} bytes, ${RX_ERRS} errors, ${RX_DROP} dropped"
    [[ "${SPEED}" == "100Mb/s" ]] && note "(100 Mb/s expected for the modified cable in this build)"
    [[ "${RX_ERRS}" != "0" || "${RX_DROP}" != "0" ]] && warn "lifetime errors/drops are non-zero; investigate"
else
    fail "no carrier; cable unplugged or peer offline"
    fix  "plug the LiDAR cable into ${NIC} and confirm the LiDAR is powered"
    exit 1
fi

step "2. NetworkManager profile on ${NIC}"
what     "Which connection is currently providing IPv4 to ${NIC}?"
why      "'rslidar' = static IP for LiDAR. 'dev' = DHCP fallback (no LiDAR detected at last link-up)."

NM_LINE=$(nmcli -t -f NAME,DEVICE,STATE,UUID connection show --active 2>/dev/null | awk -F: -v ifc="${NIC}" '$2==ifc')
raw "${NM_LINE:-(none)}"

ACTIVE=$(echo "${NM_LINE}" | awk -F: '{print $1}')
if [[ "${ACTIVE}" == "rslidar" ]]; then
    ok "active: 'rslidar' (static LiDAR profile)"
elif [[ "${ACTIVE}" == "dev" ]]; then
    warn "active: 'dev' (DHCP fallback)"
    note "the dispatcher gave up waiting for the LiDAR at the last link-up event."
    fix  "nmcli connection up rslidar"
elif [[ -n "${ACTIVE}" ]]; then
    warn "active: '${ACTIVE}' (not one of the two managed profiles)"
    fix  "nmcli connection up rslidar"
else
    fail "no NM connection active on ${NIC}"
    fix  "nmcli connection up rslidar"
fi

# Also report what other NM connections EXIST (helps diagnose missing profiles)
ALL_PROFILES=$(nmcli -t -f NAME,DEVICE connection show 2>/dev/null | awk -F: -v ifc="${NIC}" '$2==ifc {print $1}')
note "profiles bound to ${NIC}: $(echo "${ALL_PROFILES}" | tr '\n' ' ' | sed 's/ $//')"

step "3. IPv4 address + route via ${NIC}"
what     "Is there an IP on ${NIC}, and does the kernel know to send ${SUBNET} traffic out it?"
why      "Without a route, LiDAR-bound packets are silently sent via the wifi default gateway instead."

IP_LINE=$(ip -4 -o addr show "${NIC}" 2>/dev/null | head -1)
raw "${IP_LINE:-(no IPv4)}"
ASSIGNED=$(echo "${IP_LINE}" | awk '{print $4}')
if [[ -z "${ASSIGNED}" ]]; then
    fail "no IPv4 address on ${NIC}"
elif [[ "${ASSIGNED%/*}" == "${HOST_IP}" ]]; then
    ok "address: ${ASSIGNED} (matches expected ${HOST_IP})"
else
    warn "address: ${ASSIGNED} (differs from expected ${HOST_IP})"
fi
echo "${IP_LINE}" | grep -q 'noprefixroute' && \
    note "'noprefixroute' flag is set; connected route must come from ipv4.routes"

ROUTES=$(ip -4 route show | awk -v ifc="${NIC}" '$0 ~ "dev " ifc')
raw "${ROUTES:-(no routes via ${NIC})}"
if [[ -n "${ROUTES}" ]] && echo "${ROUTES}" | grep -qF "${SUBNET}"; then
    ok "route to ${SUBNET} exists via ${NIC}"
elif [[ -n "${ROUTES}" ]]; then
    warn "routes exist on ${NIC} but none for ${SUBNET}"
    fix  "nmcli connection modify rslidar +ipv4.routes '${SUBNET} 0.0.0.0' && nmcli connection up rslidar"
else
    fail "no routes via ${NIC}: kernel cannot reach ${SUBNET} on this NIC"
    fix  "nmcli connection modify rslidar +ipv4.routes '${SUBNET} 0.0.0.0' && nmcli connection up rslidar"
fi

step "4. ARP probe at ${EXPECTED_LIDAR_IP}"
what     "Send 3 ARP-Who-Has from source ${HOST_IP} on ${NIC}. Healthy LiDAR responds in <1 ms."
why      "ARP is L2; works regardless of UDP listeners. No reply ⇒ LiDAR off, at different IP, or not on this segment."

ARP_OUT=$(arping -c 3 -w 3 -s "${HOST_IP}" -I "${NIC}" "${EXPECTED_LIDAR_IP}" 2>&1)
raw "${ARP_OUT}"

if echo "${ARP_OUT}" | grep -qE 'Received [1-9]'; then
    MAC=$(echo "${ARP_OUT}" | grep -oE '([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}' | head -1)
    [[ -z "${MAC}" ]] && MAC=$(ip neigh show "${EXPECTED_LIDAR_IP}" dev "${NIC}" 2>/dev/null | awk '{print $5; exit}')
    AVG_RTT=$(echo "${ARP_OUT}" | grep -oE '[0-9.]+ms' | head -1)
    ok "LiDAR responded at ${EXPECTED_LIDAR_IP}"
    note "MAC:         ${MAC:-?}"
    note "RTT (first): ${AVG_RTT:-?}"
    if [[ -n "${EXPECTED_LIDAR_MAC}" ]]; then
        if [[ "${MAC,,}" == "${EXPECTED_LIDAR_MAC,,}" ]]; then
            note "MAC matches  ${EXPECTED_LIDAR_MAC} from the detected-config file"
        else
            warn "MAC differs from ${EXPECTED_LIDAR_MAC} in the detected-config file"
            fix  "a different LiDAR was swapped in. run 'sudo config_lidar' to re-detect."
        fi
    fi
    LIDAR_FOUND=1
    LIDAR_SRC_IP="${EXPECTED_LIDAR_IP}"
else
    warn "no ARP reply from ${EXPECTED_LIDAR_IP}"
    note "continuing to passive sniff to find where the LiDAR actually is."
fi

if [[ -z "${LIDAR_FOUND}" ]]; then
    step "5. Passive sniff (5 s) for RoboSense traffic on ${NIC}"
    what     "tcpdump in promisc mode for UDP packets on ${EXPECTED_MSOP}/${EXPECTED_DIFOP}/${EXPECTED_IMU}."
    why      "Powered LiDARs auto-broadcast MSOP/DIFOP regardless of host config. Reveals real src+dst IPs."

    if ! command -v tcpdump >/dev/null 2>&1; then
        warn "tcpdump not installed"
        fix  "sudo apt install -y tcpdump"
    elif [[ "${SUDO_OK}" != "yes" ]]; then
        warn "sudo not cached"
        fix  "re-run with sudo: sudo $0"
    else
        SNIFF_OUT=$(sudo -n timeout 5 tcpdump -i "${NIC}" -nn -c 20 \
            "udp and (port ${EXPECTED_MSOP} or port ${EXPECTED_DIFOP} or port ${EXPECTED_IMU})" 2>&1)
        raw "${SNIFF_OUT}"

        PKT_COUNT=$(echo "${SNIFF_OUT}" | grep -cE '^[0-9]+:[0-9]+:[0-9]+\.[0-9]+ IP ')
        note "packets matching MSOP/DIFOP/IMU ports in 5 s: ${PKT_COUNT}"

        if [[ "${PKT_COUNT}" -gt 0 ]]; then
            FIRST_LINE=$(echo "${SNIFF_OUT}" | grep -E '^[0-9]+:[0-9]+:[0-9]+\.[0-9]+ IP ' | head -1)
            LIDAR_SRC_IP=$(echo "${FIRST_LINE}" | awk '{print $3}' | sed -E 's/\.[0-9]+$//')
            LIDAR_DST_TUPLE=$(echo "${FIRST_LINE}" | awk '{print $5}' | sed 's/:.*//')
            LIDAR_DST_IP=$(echo "${LIDAR_DST_TUPLE}" | sed -E 's/\.[0-9]+$//')
            LIDAR_DST_PORT=$(echo "${LIDAR_DST_TUPLE}" | awk -F. '{print $NF}')
            ok "LiDAR detected via passive sniff"
            note "source IP (LiDAR):     ${LIDAR_SRC_IP}"
            note "destination IP (host): ${LIDAR_DST_IP}"
            note "destination port:      ${LIDAR_DST_PORT}"
            note "approx packets/sec:    $((PKT_COUNT/5))"
            LIDAR_FOUND=1
        else
            warn "no MSOP/DIFOP/IMU traffic seen"
            note "(a) LiDAR powered off, or"
            note "(b) wired to different NIC/cable, or"
            note "(c) MSOP port reconfigured to non-standard value"
        fi
    fi
fi

if [[ -z "${LIDAR_FOUND}" ]]; then
    step "6. Active subnet scan with arp-scan"
    what     "ARP every host in ${SUBNET} to find any responder."
    why      "Catches a silent LiDAR (rare) or other devices that might be on the wire."

    if ! command -v arp-scan >/dev/null 2>&1; then
        warn "arp-scan not installed"
        fix  "sudo apt install -y arp-scan"
    elif [[ "${SUDO_OK}" != "yes" ]]; then
        warn "sudo not cached"
        fix  "re-run with sudo: sudo $0"
    else
        SCAN_OUT=$(sudo -n arp-scan -l -I "${NIC}" 2>&1 || true)
        raw "${SCAN_OUT}"
        HITS=$(echo "${SCAN_OUT}" | awk '/^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+/')
        if [[ -n "${HITS}" ]]; then
            ok "devices found:"
            echo "${HITS}" | sed 's/^/         /'
            note "if a candidate LiDAR IP appears, run 'sudo config_lidar' to reconfigure."
        else
            warn "no devices found on ${SUBNET}"
        fi
    fi
fi

step "7. rslidar_coordinator + SDK runtime status"
what     "Is the supervisor active? Is the SDK subprocess spawned? Is /rslidar_points publishing?"
why      "Network can be perfect but the SDK might not be running, or might be running with no consumers."

SVC_STATE=$(systemctl is-active rslidar_coordinator.service 2>&1)
raw "rslidar_coordinator.service: ${SVC_STATE}"
if [[ "${SVC_STATE}" != "active" ]]; then
    warn "rslidar_coordinator.service is not active"
    fix  "sudo systemctl restart rslidar_coordinator.service"
else
    ok "rslidar_coordinator.service active"
fi

SDK_PID=$(pgrep -f rslidar_sdk_node | head -1)
if [[ -n "${SDK_PID}" ]]; then
    PORTS=$(ss -ulnp 2>/dev/null | awk -v p="${SDK_PID}" '$0 ~ "pid="p"," {print $5}' | awk -F: '{print $NF}' | sort -u | tr '\n' ' ')
    info "rslidar_sdk_node: pid=${SDK_PID}, bound UDP ports: ${PORTS}"

    # ROS env may not be loaded in this shell; only check topics if ros2 is on PATH
    if command -v ros2 >/dev/null 2>&1; then
        # Sample /rslidar_points for 3 s
        COUNT=$(timeout 3 ros2 topic echo --no-arr --qos-reliability best_effort /rslidar_points 2>/dev/null | grep -c '^---$')
        HZ=$(awk "BEGIN{printf \"%.1f\", $COUNT/3}")
        note "/rslidar_points: ${COUNT} msgs / 3 s ≈ ${HZ} Hz"
        # alive Bool
        ALIVE=$(timeout 4 ros2 topic echo --once \
            --qos-reliability reliable --qos-durability transient_local --qos-depth 1 \
            /rslidar_coordinator/alive 2>/dev/null | grep -oE 'data: (true|false)' | head -1)
        note "/rslidar_coordinator/alive: ${ALIVE:-(no value yet)}"
    else
        note "ros2 CLI not in PATH; skipping topic-level checks (source local_ws/install/setup.bash first)"
    fi
else
    info "no rslidar_sdk_node subprocess running"
    fix  "rslidar_start  # spawns it via the coordinator"
fi

hdr "Summary"
if [[ -n "${LIDAR_FOUND}" ]]; then
    echo "  LiDAR source IP:        ${LIDAR_SRC_IP:-?}"
    [[ -n "${LIDAR_DST_IP}"   ]] && echo "  LiDAR destination IP:   ${LIDAR_DST_IP}  (the host must hold this)"
    [[ -n "${LIDAR_DST_PORT}" ]] && echo "  LiDAR destination port: ${LIDAR_DST_PORT}"
    echo ""

    if [[ "${LIDAR_SRC_IP}" == "${EXPECTED_LIDAR_IP}" ]] && \
       ( [[ -z "${LIDAR_DST_IP}" ]] || [[ "${LIDAR_DST_IP}" == "${HOST_IP}" ]] ); then
        ok "Network correctly configured."
        [[ -z "${SDK_PID}" ]] && note "run 'rslidar_start' to bring up the SDK and start publishing /rslidar_points"
    else
        warn "Network needs adjustment:"
        [[ -n "${LIDAR_SRC_IP}" && "${LIDAR_SRC_IP}" != "${EXPECTED_LIDAR_IP}" ]] && \
            note "  • LiDAR's source IP (${LIDAR_SRC_IP}) ≠ expected (${EXPECTED_LIDAR_IP})"
        [[ -n "${LIDAR_DST_IP}" && "${LIDAR_DST_IP}" != "${HOST_IP}" ]] && \
            note "  • LiDAR sends to ${LIDAR_DST_IP}, host is at ${HOST_IP}"
        fix  "sudo config_lidar  # re-sniffs and reconfigures both the NM profile and the dispatcher"
    fi
else
    fail "LiDAR not detected"
    echo ""
    echo "  Check, in order:"
    echo "    1. LiDAR power:       status LED on?"
    echo "    2. Cable destination: wired to the LiDAR's Ethernet port, not power/debug?"
    echo "    3. LiDAR IP:          RSView or factory-reset to confirm. RSAIRY default is"
    echo "                          ${EXPECTED_LIDAR_IP} -> ${HOST_IP} on UDP ${EXPECTED_MSOP}/${EXPECTED_DIFOP}."
fi
echo ""
