#!/bin/bash
# Auto-detect the RSAIRY LiDAR on enP8p1s0 by sniffing ARP/UDP, then rewrite the NetworkManager
# 'rslidar' profile and the dispatcher to match it. RoboSense stores its IP config in firmware,
# so a prior RSView session leaves the unit off the factory defaults permanently.

set -u

NIC="enP8p1s0"
SNIFF_SECS=10
ROBOSENSE_OUI="40:2c:76"          # search preference only: a non-matching OUI is still accepted
DISPATCHER="/etc/NetworkManager/dispatcher.d/90-rslidar"
WORKSPACES="${WORKSPACES:-/home/jetson/workspaces}"
LIDAR_STATE_DIR="${WORKSPACES}/.lidar"
DETECTED_CONF="${LIDAR_STATE_DIR}/rslidar_detected.conf"
STATE_OWNER="${SUDO_USER:-jetson}"

if [[ -t 1 ]]; then
    BLUE='\033[1;34m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
else
    BLUE=''; GREEN=''; YELLOW=''; RED=''; CYAN=''; BOLD=''; NC=''
fi
hdr()  { echo -e "\n${BLUE}${BOLD}================================================================================${NC}"; echo -e "${BLUE}${BOLD}$*${NC}"; echo -e "${BLUE}${BOLD}================================================================================${NC}"; }
step() { echo -e "\n${BLUE}${BOLD}-- $* --${NC}"; }
what() { echo -e "${CYAN}WHAT:${NC}    $*"; }
why()  { echo -e "${CYAN}WHY:${NC}     $*"; }
raw()  { echo -e "${CYAN}RAW:${NC}"; echo "$1" | sed 's/^/         /'; }
ok()   { echo -e "${GREEN}RESULT:  [OK]    $*${NC}"; }
warn() { echo -e "${YELLOW}RESULT:  [WARN]  $*${NC}"; }
fail() { echo -e "${RED}RESULT:  [FAIL]  $*${NC}"; }
fix()  { echo -e "${CYAN}FIX:${NC}     $*"; }
note() { echo -e "         $*"; }

if [[ $EUID -ne 0 ]]; then
    fail "config_lidar.sh must run as root (uses tcpdump promisc + writes /etc files)."
    fix  "sudo $0     # or:  config_lidar    (the alias prepends sudo)"
    exit 2
fi

hdr "RSAIRY LiDAR Auto-Configuration"
echo ""
echo "Host:     $(hostname)"
echo "NIC:      ${NIC}"
echo "OUI:      ${ROBOSENSE_OUI}:xx:xx:xx (RoboSense)"
echo "Sniff:    ${SNIFF_SECS} s on ${NIC}"
echo "Output:   ${DETECTED_CONF}"

step "1. Carrier check on ${NIC}"
what     "Verify the cable is plugged in and the link partner is electrically alive."
why      "Without carrier there's no possible traffic to sniff."

if ! ip link show "${NIC}" 2>/dev/null | grep -qE 'LOWER_UP'; then
    LINK_LINE=$(ip -o link show "${NIC}" 2>&1 | head -1)
    raw "${LINK_LINE}"
    fail "no carrier on ${NIC}"
    fix  "plug in the LiDAR cable and confirm the LiDAR is powered"
    exit 1
fi
SPEED=$(ethtool "${NIC}" 2>/dev/null | awk -F: '/Speed:/ {gsub(/^[ \t]+/,"",$2); print $2}')
ok "carrier UP - ${SPEED:-?}"

step "2. Passive sniff (${SNIFF_SECS} s) on ${NIC}"
what     "Capture packets with -nn -e and filter post-hoc for the RoboSense OUI."
why      "Any RoboSense LiDAR either ARPs for its configured host (giving both IPs cleanly) or unicasts MSOP/DIFOP UDP packets (giving source + destination). One sniff reveals which."

CAP=$(mktemp /tmp/lidar_sniff.XXXXXX)
trap 'rm -f "$CAP"' EXIT

BPF='arp or (udp and (port 6699 or port 7788 or port 6688))'
timeout "${SNIFF_SECS}" tcpdump -i "${NIC}" -nn -e -l "${BPF}" 2>/dev/null > "${CAP}" || true

LINE_COUNT=$(wc -l < "${CAP}")
ROBO_LINES=$(grep -c -i "${ROBOSENSE_OUI}" "${CAP}" || true)
note "total interesting packets captured: ${LINE_COUNT}"
note "lines matching RoboSense OUI (${ROBOSENSE_OUI}): ${ROBO_LINES}"

if [[ "${LINE_COUNT}" -eq 0 ]]; then
    fail "nothing seen on ${NIC} in ${SNIFF_SECS} s: no ARP, no UDP on LiDAR ports"
    note "LiDAR is off, cable not reaching it, or destination ports are reconfigured to non-standard values."
    exit 1
fi

step "3. Extract LiDAR IP, MAC, and the host IP it wants"
what     "Prefer ARP lines (cleanest), fall back to UDP src/dst extraction."
why      "ARP gives 'tell <LiDAR_IP>' and 'who-has <HOST_IP>' in one line. UDP gives 'SRC_IP.PORT > DST_IP.PORT'."

LIDAR_MAC=""
LIDAR_IP=""
HOST_IP=""
MATCH_HOW=""

SELF_MAC=$(cat "/sys/class/net/${NIC}/address" 2>/dev/null | tr 'A-Z' 'a-z')
SELF_IPS=$(ip -4 -o addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1)

# The capture is promiscuous and undirected, so it also holds this host's own arping probes.
# Accepting one as the LiDAR writes the host's own address into rslidar_detected.conf and into
# the 90-rslidar dispatcher; the dispatcher then arpings that address, never gets a reply, and
# drops the static profile to DHCP on every link-up, leaving the LiDAR unreachable.
candidate_is_self() {
    local mac="$1" lidar_ip="$2" host_ip="$3" ip
    [[ -z "$lidar_ip" || -z "$host_ip" ]]          && return 0   # unusable -> treat as self
    [[ "$lidar_ip" == "$host_ip" ]]                && return 0   # talking to itself
    [[ -n "$mac" && "${mac,,}" == "${SELF_MAC}" ]] && return 0   # our own NIC sent it
    for ip in ${SELF_IPS}; do
        [[ "$lidar_ip" == "$ip" ]] && return 0                   # "LiDAR" is an address we hold
    done
    return 1
}

# Both extractors walk every matching line rather than taking the first: the first is frequently
# this host's own probe, and the next valid line is the LiDAR.
extract_from_arp() {
    local pat="$1" line mac lip hip
    while IFS= read -r line; do
        [[ -z "$line" ]] && continue
        mac=$(echo "$line" | grep -oE '([0-9a-f]{2}:){5}[0-9a-f]{2}' | head -1)
        hip=$(echo "$line" | grep -oE 'who-has [0-9.]+' | awk '{print $2}')
        lip=$(echo "$line" | grep -oE 'tell [0-9.]+'    | awk '{print $2}')
        candidate_is_self "$mac" "$lip" "$hip" && continue
        LIDAR_MAC="$mac"; LIDAR_IP="$lip"; HOST_IP="$hip"
        return 0
    done < <(grep -E "$pat" "${CAP}" | grep -i "who-has.*tell")
    return 1
}
extract_from_udp() {
    local pat="$1" line tuple mac lip hip
    while IFS= read -r line; do
        [[ -z "$line" ]] && continue
        mac=$(echo "$line" | grep -oE '([0-9a-f]{2}:){5}[0-9a-f]{2}' | head -1)
        tuple=$(echo "$line" | grep -oE '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+ > [0-9]+\.[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' | head -1)
        [[ -z "$tuple" ]] && continue
        lip=$(echo "$tuple" | awk '{print $1}' | sed -E 's/\.[0-9]+$//')
        hip=$(echo "$tuple" | awk '{print $3}' | sed -E 's/\.[0-9]+$//')
        candidate_is_self "$mac" "$lip" "$hip" && continue
        LIDAR_MAC="$mac"; LIDAR_IP="$lip"; HOST_IP="$hip"
        return 0
    done < <(grep -E "$pat" "${CAP}" | grep -E 'UDP, length')
    return 1
}

if   extract_from_arp "${ROBOSENSE_OUI}";    then MATCH_HOW="ARP from RoboSense OUI"
elif extract_from_udp "${ROBOSENSE_OUI}";    then MATCH_HOW="UDP from RoboSense OUI"
elif extract_from_arp '.';                   then MATCH_HOW="ARP from non-RoboSense OUI on a LiDAR port"
elif extract_from_udp '.';                   then MATCH_HOW="UDP on a LiDAR port (any OUI)"
fi

raw "$(head -3 "${CAP}")"

if [[ -z "${LIDAR_IP}" || -z "${HOST_IP}" ]]; then
    fail "traffic present but couldn't extract LiDAR/host IPs"
    note "neither ARP-resolve nor UDP-with-IPs found in the capture."
    note "LiDAR may be emitting only PTP (L2 only); RSView setup needed to get it ARPing/unicasting."
    exit 1
fi

if candidate_is_self "${LIDAR_MAC}" "${LIDAR_IP}" "${HOST_IP}"; then
    fail "detection matched this host, not the LiDAR (LiDAR ${LIDAR_IP} / host ${HOST_IP}, MAC ${LIDAR_MAC:-none})"
    note "this machine is ${SELF_MAC} holding: $(echo ${SELF_IPS} | tr '\n' ' ')"
    note "the capture likely contained only our own ARP probes - the LiDAR may be powered off,"
    note "on a different segment, or emitting L2-only PTP. Nothing was written."
    exit 1
fi

SUBNET="$(echo "${LIDAR_IP}" | awk -F. '{print $1"."$2"."$3".0/24"}')"
ok "discovered LiDAR config (via: ${MATCH_HOW}):"
note "LiDAR MAC: ${LIDAR_MAC}"
note "LiDAR IP:  ${LIDAR_IP}"
note "Host IP:   ${HOST_IP}     (the IP the LiDAR ARPs/unicasts to)"
note "Subnet:    ${SUBNET}"

step "4. Reconfigure NetworkManager 'rslidar' connection in place"
what     "Update ipv4.addresses + ipv4.routes on the existing connection profile."
why      "The LiDAR ARPs for the host IP; without that address on ${NIC} the request cannot be resolved. ipv4.routes is also required because NetworkManager sets noprefixroute on manual addresses."

if ! nmcli -t -f NAME connection show 2>/dev/null | grep -qxF "rslidar"; then
    warn "NM connection 'rslidar' does not exist yet; creating it"
    nmcli connection add type ethernet con-name rslidar ifname "${NIC}" \
        ipv4.method manual \
        ipv4.addresses "${HOST_IP}/24" \
        ipv4.routes "${SUBNET} 0.0.0.0" \
        autoconnect yes \
        connection.autoconnect-priority 10 >/dev/null
    ok "created 'rslidar' with ${HOST_IP}/24 + route ${SUBNET}"
else
    nmcli connection modify rslidar \
        ipv4.method manual \
        ipv4.addresses "${HOST_IP}/24" \
        ipv4.routes "${SUBNET} 0.0.0.0"
    ok "modified 'rslidar' to ${HOST_IP}/24 + route ${SUBNET}"
fi

step "5. Rewrite ${DISPATCHER}"
what     "Replace the dispatcher's hardcoded LiDAR IP + source IP with the discovered values."
why      "Otherwise on the next link-up the dispatcher will arping the wrong IP, time out, and drop to DHCP fallback."

cat > "${DISPATCHER}" << EOL
#!/bin/bash
# Auto-generated by config_lidar.sh on $(date -u +%Y-%m-%dT%H:%M:%SZ)
# Detected LiDAR: ${LIDAR_IP} (MAC ${LIDAR_MAC}), expects host at ${HOST_IP}
IFACE="\$1"
ACTION="\$2"

[[ "\$IFACE" != "${NIC}" ]] && exit 0
[[ "\$ACTION" != "up" ]] && exit 0

ACTIVE=\$(nmcli -t -f NAME connection show --active 2>/dev/null | grep -xE 'rslidar|dev' | head -1)
[[ "\$ACTIVE" != "rslidar" ]] && exit 0

# -s pins the ARP source IP; without it the kernel may pick a wifi IP.
for _ in 1 2 3 4 5 6 7 8; do
    if arping -c 1 -w 1 -s ${HOST_IP} -I "\$IFACE" ${LIDAR_IP} >/dev/null 2>&1; then
        logger -t rslidar-net "LiDAR detected at ${LIDAR_IP}; staying on static."
        exit 0
    fi
done

logger -t rslidar-net "LiDAR not detected after 8 s; switching to DHCP."
nmcli connection down rslidar >/dev/null 2>&1 || true
nmcli connection up dev >/dev/null 2>&1 || true
EOL
chmod 755 "${DISPATCHER}"
chown root:root "${DISPATCHER}"
ok "dispatcher rewritten"

step "6. Bring up rslidar"
what     "Deactivate first if running, then reactivate so NM applies the new IP + route."

nmcli connection down rslidar >/dev/null 2>&1 || true
if nmcli connection up rslidar >/dev/null 2>&1; then
    ok "rslidar activated"
else
    fail "couldn't activate rslidar: check 'nmcli connection show'"
    exit 1
fi

sleep 2
note "current state:"
note "  ip:    $(ip -4 -o addr show "${NIC}" 2>/dev/null | awk '{print $4}')"
note "  route: $(ip -4 route show | awk -v ifc="${NIC}" '$0 ~ "dev " ifc {print; exit}')"

step "7. Verify LiDAR responds at ${LIDAR_IP}"
what     "ARP-probe the LiDAR. With the new host IP in place, it should respond within ~100 ms."

if arping -c 3 -w 3 -s "${HOST_IP}" -I "${NIC}" "${LIDAR_IP}" >/dev/null 2>&1; then
    LEARNED_MAC=$(ip neigh show "${LIDAR_IP}" dev "${NIC}" 2>/dev/null | awk '{print $5; exit}')
    ok "LiDAR responding at ${LIDAR_IP} (MAC ${LEARNED_MAC:-?})"
else
    warn "no ARP reply after reconfigure"
    note "the LiDAR may still be cold-booting; try again in 10-15 s or check power"
fi

step "8. Persist discovery to ${DETECTED_CONF}"
what     "Write the discovered values so other tooling (setup.sh, diag scripts) can read them."
why      "Local-to-workspace, gitignored. Owned by ${STATE_OWNER} so it stays editable without sudo."

mkdir -p "${LIDAR_STATE_DIR}"
chown "${STATE_OWNER}:${STATE_OWNER}" "${LIDAR_STATE_DIR}" 2>/dev/null || true

cat > "${DETECTED_CONF}" << EOF
# Auto-generated by config_lidar.sh
# Last detected: $(date -u +%Y-%m-%dT%H:%M:%SZ)
# Detection path: ${MATCH_HOW}
RSLIDAR_NIC="${NIC}"
RSLIDAR_HOST_IP="${HOST_IP}"
RSLIDAR_LIDAR_IP="${LIDAR_IP}"
RSLIDAR_LIDAR_MAC="${LIDAR_MAC}"
RSLIDAR_SUBNET="${SUBNET}"
EOF
chmod 644 "${DETECTED_CONF}"
chown "${STATE_OWNER}:${STATE_OWNER}" "${DETECTED_CONF}" 2>/dev/null || true
ok "wrote ${DETECTED_CONF}"

hdr "Summary"
echo "  LiDAR detected at:   ${LIDAR_IP}  (MAC ${LIDAR_MAC})"
echo "  Host configured as:  ${HOST_IP}/24"
echo "  Subnet:              ${SUBNET}"
echo ""
echo "  Next step: 'rslidar_start' to spawn the SDK and start receiving point clouds."
echo ""
exit 0
