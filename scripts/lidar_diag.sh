#!/bin/bash
# lidar_diag.sh — discover the RSAIRY LiDAR on enP8p1s0 and verify the host's
# network configuration for it.
#
# Verbose-by-design: at every step this prints WHAT is being checked, the RAW
# DATA it found, WHY it matters, and (if applicable) WHAT TO DO. Re-runnable.
# Some steps (tcpdump, arp-scan) need sudo; the rest work as a normal user.

set -u

NIC="enP8p1s0"
EXPECTED_LIDAR_IP="192.168.1.200"
HOST_IP="192.168.1.102"
SUBNET="192.168.1.0/24"
EXPECTED_MSOP=6699
EXPECTED_DIFOP=7788

# ───────────────── output helpers ─────────────────
if [[ -t 1 ]]; then
    BLUE='\033[1;34m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
else
    BLUE=''; GREEN=''; YELLOW=''; RED=''; CYAN=''; BOLD=''; NC=''
fi
hdr()      { echo -e "\n${BLUE}${BOLD}================================================================================${NC}"; echo -e "${BLUE}${BOLD}$*${NC}"; echo -e "${BLUE}${BOLD}================================================================================${NC}"; }
step()     { echo -e "\n${BLUE}${BOLD}── $* ──${NC}"; }
what()     { echo -e "${CYAN}WHAT:${NC}    $*"; }
why()      { echo -e "${CYAN}WHY:${NC}     $*"; }
raw()      { echo -e "${CYAN}RAW:${NC}"; echo "$1" | sed 's/^/         /'; }
ok()       { echo -e "${GREEN}RESULT:  [OK]    $*${NC}"; }
warn()     { echo -e "${YELLOW}RESULT:  [WARN]  $*${NC}"; }
fail()     { echo -e "${RED}RESULT:  [FAIL]  $*${NC}"; }
fix()      { echo -e "${CYAN}FIX:${NC}     $*"; }
note()     { echo -e "         $*"; }

LIDAR_FOUND=""
LIDAR_SRC_IP=""
LIDAR_DST_IP=""
LIDAR_DST_PORT=""

hdr "RSAIRY LiDAR Network Diagnostic"
echo ""
echo "Host:           $(hostname)"
echo "User:           $(id -un)"
echo "NIC under test: ${NIC}"
echo "Expected setup: LiDAR at ${EXPECTED_LIDAR_IP}, host at ${HOST_IP}/24"
echo "                MSOP UDP/${EXPECTED_MSOP} (point cloud), DIFOP UDP/${EXPECTED_DIFOP} (device info)"
echo "Sudo cached:    $(sudo -n true 2>/dev/null && echo yes || echo no  '(passive sniff + arp-scan will be skipped if no)')"

# ───────────────── 1. interface presence + link state ─────────────────
step "1. Interface presence + electrical link"
what     "Does the kernel see interface ${NIC}? Is the cable carrying signal (LOWER_UP)?"
why      "No interface = ARK PAB Ethernet not enumerated (kernel/driver problem)."
why      "No carrier   = cable unplugged, peer offline, or a broken cable."

if ! ip link show "${NIC}" >/dev/null 2>&1; then
    fail "interface ${NIC} not present on this host"
    fix  "check 'ip -o link show' to find the actual ARK PAB NIC name"
    exit 1
fi
LINK_LINE=$(ip -o link show "${NIC}")
raw "${LINK_LINE}"

if echo "${LINK_LINE}" | grep -qE 'LOWER_UP'; then
    SPEED=$(ethtool "${NIC}" 2>/dev/null | awk -F: '/Speed:/ {gsub(/^[ \t]+/, "", $2); print $2}')
    DUPLEX=$(ethtool "${NIC}" 2>/dev/null | awk -F: '/Duplex:/ {gsub(/^[ \t]+/, "", $2); print $2}')
    MTU=$(echo "${LINK_LINE}" | awk '{for(i=1;i<=NF;i++) if($i=="mtu") print $(i+1)}')
    ok "carrier UP — negotiated ${SPEED:-?}, ${DUPLEX:-?}, MTU ${MTU:-?}"
    [[ "${SPEED}" == "100Mb/s" ]] && note "(100 Mb is expected for the modified cable in this build.)"
else
    fail "no carrier — cable unplugged or peer offline"
    fix  "verify the LiDAR is wired in and powered on"
    exit 1
fi

# ───────────────── 2. NetworkManager active profile ─────────────────
step "2. Active NetworkManager profile on ${NIC}"
what     "Which NM connection profile is currently providing IPv4 to ${NIC}?"
why      "'rslidar' = static 192.168.1.102 (LiDAR mode). 'dev' = DHCP fallback (router mode)."

NM_LINE=$(nmcli -t -f NAME,DEVICE,STATE,UUID connection show --active 2>/dev/null | awk -F: -v ifc="${NIC}" '$2==ifc')
raw "${NM_LINE:-(none)}"

ACTIVE=$(echo "${NM_LINE}" | awk -F: '{print $1}')
if [[ "${ACTIVE}" == "rslidar" ]]; then
    ok "active: 'rslidar' (static profile) — correct for talking to the LiDAR"
elif [[ "${ACTIVE}" == "dev" ]]; then
    warn "active: 'dev' (DHCP profile)"
    note "the dispatcher fell back because the LiDAR didn't reply to ARP at link-up."
    fix  "force back to static with: nmcli connection up rslidar"
elif [[ -n "${ACTIVE}" ]]; then
    warn "active: '${ACTIVE}' — unexpected; not one of our two managed profiles"
else
    fail "no NM connection active on ${NIC}"
    fix  "nmcli connection up rslidar"
fi

# ───────────────── 3. IPv4 address + route ─────────────────
step "3. IPv4 address + route via ${NIC}"
what     "Does the kernel actually have an IP on ${NIC}, and a route to ${SUBNET}?"
why      "Without a route, the kernel sends LiDAR-bound packets out wifi instead — silently broken."
why      "Without an IP, nothing on ${SUBNET} is reachable."

IP_LINE=$(ip -4 -o addr show "${NIC}" 2>/dev/null | head -1)
raw "${IP_LINE:-(no IPv4)}"
ASSIGNED=$(echo "${IP_LINE}" | awk '{print $4}')
if [[ -z "${ASSIGNED}" ]]; then
    fail "no IPv4 address on ${NIC}"
elif [[ "${ASSIGNED}" == "${HOST_IP}/24" ]]; then
    ok "address: ${ASSIGNED} (matches expected ${HOST_IP}/24)"
else
    warn "address: ${ASSIGNED} (differs from expected ${HOST_IP}/24)"
fi
echo "${IP_LINE}" | grep -q 'noprefixroute' && \
    note "'noprefixroute' flag is set — NM suppressed the auto-route, so the route MUST come from ipv4.routes"

ROUTES=$(ip -4 route show | awk -v ifc="${NIC}" '$0 ~ "dev " ifc')
raw "${ROUTES:-(no routes via ${NIC})}"
if [[ -n "${ROUTES}" ]] && echo "${ROUTES}" | grep -qF "${SUBNET}"; then
    ok "route to ${SUBNET} exists via ${NIC}"
elif [[ -n "${ROUTES}" ]]; then
    warn "routes exist on ${NIC} but none for ${SUBNET}"
    fix  "nmcli connection modify rslidar +ipv4.routes '${SUBNET} 0.0.0.0' && nmcli connection up rslidar"
else
    fail "no routes via ${NIC} — kernel cannot reach ${SUBNET} on this NIC"
    fix  "nmcli connection modify rslidar +ipv4.routes '${SUBNET} 0.0.0.0' && nmcli connection up rslidar"
fi

# ───────────────── 4. ARP probe at expected LiDAR IP ─────────────────
step "4. ARP probe at the expected LiDAR address (${EXPECTED_LIDAR_IP})"
what     "Send 3 ARP requests asking 'who has ${EXPECTED_LIDAR_IP}?' from source ${HOST_IP} on ${NIC}."
why      "A powered, network-reachable RoboSense LiDAR replies to ARP within ~100 ms."
why      "ARP works at L2 — independent of any UDP listener. If it doesn't reply here, the LiDAR is off, at a different IP, or not on this segment."

ARP_OUT=$(arping -c 3 -w 3 -s "${HOST_IP}" -I "${NIC}" "${EXPECTED_LIDAR_IP}" 2>&1)
raw "${ARP_OUT}"

if echo "${ARP_OUT}" | grep -qE 'Received [1-9]'; then
    MAC=$(ip neigh show "${EXPECTED_LIDAR_IP}" dev "${NIC}" 2>/dev/null | awk '{print $5; exit}')
    ok "LiDAR responded — MAC ${MAC:-unknown}"
    LIDAR_FOUND=1
    LIDAR_SRC_IP="${EXPECTED_LIDAR_IP}"
else
    warn "no ARP reply from ${EXPECTED_LIDAR_IP}"
    note "continuing with passive sniff to see if the LiDAR is at a different IP"
fi

# ───────────────── 5. passive sniff for MSOP/DIFOP ─────────────────
if [[ -z "${LIDAR_FOUND}" ]]; then
    step "5. Passive sniff (5 s) — listen for any RoboSense traffic on ${NIC}"
    what     "Run tcpdump in promisc mode, capture up to 10 UDP packets on port ${EXPECTED_MSOP}/${EXPECTED_DIFOP}."
    why      "RoboSense LiDARs auto-broadcast MSOP+DIFOP packets continuously when powered, regardless of host config."
    why      "If we see them, we learn: (a) LiDAR's actual source IP, (b) the destination IP it's configured to send to."

    if ! command -v tcpdump >/dev/null 2>&1; then
        warn "tcpdump not installed — cannot run passive sniff"
        fix  "sudo apt install -y tcpdump"
    elif ! sudo -n true 2>/dev/null; then
        warn "sudo password required but not cached — cannot run passive sniff"
        fix  "re-run with sudo for the sniff/scan steps:  sudo $0"
    else
        SNIFF_OUT=$(sudo -n timeout 5 tcpdump -i "${NIC}" -nn -c 10 "udp and (port ${EXPECTED_MSOP} or port ${EXPECTED_DIFOP})" 2>&1)
        raw "${SNIFF_OUT}"

        if echo "${SNIFF_OUT}" | grep -qE '^[0-9]+:[0-9]+:[0-9]+\.[0-9]+ IP '; then
            FIRST_LINE=$(echo "${SNIFF_OUT}" | grep -E '^[0-9]+:[0-9]+:[0-9]+\.[0-9]+ IP ' | head -1)
            LIDAR_SRC_IP=$(echo "${FIRST_LINE}" | awk '{print $3}' | sed -E 's/\.[0-9]+$//')
            LIDAR_DST_TUPLE=$(echo "${FIRST_LINE}" | awk '{print $5}' | sed 's/:.*//')
            LIDAR_DST_IP=$(echo "${LIDAR_DST_TUPLE}" | sed -E 's/\.[0-9]+$//')
            LIDAR_DST_PORT=$(echo "${LIDAR_DST_TUPLE}" | awk -F. '{print $NF}')
            ok "LiDAR detected via passive sniff"
            note "source IP (LiDAR):       ${LIDAR_SRC_IP}"
            note "destination IP (host):   ${LIDAR_DST_IP}"
            note "destination port:        ${LIDAR_DST_PORT}"
            LIDAR_FOUND=1
        else
            warn "no MSOP/DIFOP traffic seen in 5 s"
            note "interpretations:"
            note "  (a) LiDAR is powered OFF"
            note "  (b) LiDAR is wired to a different cable / NIC"
            note "  (c) LiDAR's MSOP port is reconfigured to something other than ${EXPECTED_MSOP}/${EXPECTED_DIFOP}"
        fi
    fi
fi

# ───────────────── 6. subnet scan (last resort) ─────────────────
if [[ -z "${LIDAR_FOUND}" ]]; then
    step "6. Active subnet scan with arp-scan (last resort)"
    what     "Send ARP requests to every host in ${SUBNET}."
    why      "If the LiDAR is at a non-default IP and isn't actively broadcasting (rare), arp-scan will still find it."

    if ! command -v arp-scan >/dev/null 2>&1; then
        warn "arp-scan not installed — skipping"
        fix  "sudo apt install -y arp-scan"
    elif ! sudo -n true 2>/dev/null; then
        warn "sudo password required but not cached — skipping"
        fix  "re-run with sudo:  sudo $0"
    else
        SCAN_OUT=$(sudo -n arp-scan -l -I "${NIC}" 2>&1 || true)
        raw "${SCAN_OUT}"
        HITS=$(echo "${SCAN_OUT}" | awk '/^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+/')
        if [[ -n "${HITS}" ]]; then
            ok "devices on the segment:"
            echo "${HITS}" | sed 's/^/         /'
            note "if you see a candidate LiDAR IP, update RSLIDAR_LIDAR_IP in setup.sh"
        else
            warn "no devices found on ${SUBNET}"
        fi
    fi
fi

# ───────────────── final verdict ─────────────────
hdr "Summary"
if [[ -n "${LIDAR_FOUND}" ]]; then
    echo "  LiDAR source IP:      ${LIDAR_SRC_IP:-?}"
    [[ -n "${LIDAR_DST_IP}"   ]] && echo "  LiDAR destination IP: ${LIDAR_DST_IP} (this host should hold this address)"
    [[ -n "${LIDAR_DST_PORT}" ]] && echo "  LiDAR destination port: ${LIDAR_DST_PORT}"
    echo ""

    if [[ "${LIDAR_SRC_IP}" == "${EXPECTED_LIDAR_IP}" ]] && \
       ( [[ -z "${LIDAR_DST_IP}" ]] || [[ "${LIDAR_DST_IP}" == "${HOST_IP}" ]] ); then
        ok "Network is correctly configured — run 'rslidar_start' to bring up the SDK."
    else
        warn "Network needs adjustment:"
        [[ -n "${LIDAR_SRC_IP}" && "${LIDAR_SRC_IP}" != "${EXPECTED_LIDAR_IP}" ]] && \
            note "  • LiDAR's source IP (${LIDAR_SRC_IP}) ≠ expected (${EXPECTED_LIDAR_IP}) — update RSLIDAR_LIDAR_IP in setup.sh"
        [[ -n "${LIDAR_DST_IP}" && "${LIDAR_DST_IP}" != "${HOST_IP}" ]] && \
            note "  • LiDAR sends to ${LIDAR_DST_IP}, but our host is ${HOST_IP} — either:"
        [[ -n "${LIDAR_DST_IP}" && "${LIDAR_DST_IP}" != "${HOST_IP}" ]] && \
            note "       - change RSLIDAR_HOST_IP in setup.sh to ${LIDAR_DST_IP}, OR"
        [[ -n "${LIDAR_DST_IP}" && "${LIDAR_DST_IP}" != "${HOST_IP}" ]] && \
            note "       - use RSView to reconfigure the LiDAR to send to ${HOST_IP}"
    fi
else
    fail "LiDAR not detected"
    echo ""
    echo "  Things to check, in order:"
    echo "    1. LiDAR power       — is the status LED on?"
    echo "    2. Cable destination — is it plugged into the LiDAR's Ethernet port,"
    echo "                           not its power/debug port?"
    echo "    3. LiDAR's IP        — use RSView or factory-reset to confirm. RSAIRY default is"
    echo "                           ${EXPECTED_LIDAR_IP} sending to ${HOST_IP} on UDP ${EXPECTED_MSOP}/${EXPECTED_DIFOP}."
    echo "                           (NIC is always ${NIC} — the ARK PAB built-in Ethernet.)"
fi
echo ""
