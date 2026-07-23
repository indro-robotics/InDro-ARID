#!/bin/bash
# ZeroTier network join/switch (alias: zt_join).
#
# Single-network model: joining leaves every other network first; re-joining an
# already-joined network is leave + join.
#
#   zt_join                  interactive: pick a joined network to switch to, or join a new one
#   zt_join <network-id>     non-interactive join (16 hex chars)
#   zt_join --setup <id>     setup.sh mode: same as above, no prompts, plain output
set -u

SUDO="sudo"
[[ $EUID -eq 0 ]] && SUDO=""

err()  { echo "ERROR: $*" >&2; }

ztc() { ${SUDO} zerotier-cli "$@"; }

command -v zerotier-cli >/dev/null 2>&1 || { err "zerotier-cli not installed (run setup)"; exit 1; }
if ! systemctl is-active --quiet zerotier-one 2>/dev/null; then
    ${SUDO} systemctl start zerotier-one || { err "zerotier-one service failed to start"; exit 1; }
    sleep 2
fi

SETUP_MODE=0
if [[ "${1-}" == "--setup" ]]; then
    SETUP_MODE=1
    shift
fi

valid_id() { [[ "$1" =~ ^[0-9a-fA-F]{16}$ ]]; }

# JSON output only: the plain listnetworks columns shift when the network name is
# empty or contains spaces.
zt_json() {
    ztc -j listnetworks 2>/dev/null | python3 -c "
import json, sys
try:
    nets = json.load(sys.stdin)
except ValueError:
    sys.exit(0)
for n in nets:
    ips = ','.join(n.get('assignedAddresses') or []) or '-'
    print('%s %s %s' % (n.get('nwid', '?'), n.get('status', '?'), ips))
"
}

joined_ids() { zt_json | awk '{print $1}'; }

# Leave everything except $1, then join $1. Idempotent.
switch_to() {
    local target="$1" id
    for id in $(joined_ids); do
        if [[ "$id" != "$target" ]]; then
            echo "leaving ${id}"
            ztc leave "$id" >/dev/null
        fi
    done
    # Overwrite semantics: a re-join of a current member is leave + join.
    if joined_ids | grep -q "^${target}$"; then
        ztc leave "$target" >/dev/null
        sleep 1
    fi
    echo "joining ${target}"
    ztc join "$target" >/dev/null || { err "join failed"; return 1; }
    report "$target"
}

# Poll the target network until OK (with an IP) or the tries run out. Prints one status line.
poll_status() {
    local target="$1" tries="${2:-15}" line
    STATUS=""; IP="-"
    for _ in $(seq 1 "$tries"); do
        line=$(zt_json | awk -v n="$target" '$1==n')
        STATUS=$(echo "$line" | awk '{print $2}')
        IP=$(echo "$line" | awk '{print $3}')
        [[ "$STATUS" == "OK" && "$IP" != "-" ]] && break
        sleep 2
    done
    echo "network ${target}: status=${STATUS:-unknown} ip=${IP:--}"
}

report() {
    local target="$1" node
    node=$(ztc info 2>/dev/null | awk '{print $3}')
    poll_status "$target" 5
    if [[ "${STATUS:-}" == "OK" ]]; then
        return 0
    fi
    if [[ "${STATUS:-}" != "ACCESS_DENIED" ]]; then
        echo "network ${target} did not come up (status=${STATUS:-unknown})"
        return 1
    fi
    echo ""
    echo "======================================================"
    echo "  This node's ZeroTier address:  ${node}"
    echo "  Authorize it on network ${target} in ZeroTier"
    echo "  Central (tick Auth; assign/auto-assign an IP)."
    echo "======================================================"
    # Any key re-tests, 's' skips; headless stays non-blocking so a scripted run never hangs.
    if [[ ! -r /dev/tty ]]; then
        echo "No terminal - authorize in Central, then re-run 'zt_join' to verify."
        return 1
    fi
    local k
    while true; do
        read -r -n1 -p "Press any key to test the connection, or 's' to skip: " k </dev/tty || k="s"
        echo ""
        if [[ "${k,,}" == "s" ]]; then
            echo "Skipped - authorize in Central later, then 'zt_join' to verify."
            return 1
        fi
        poll_status "$target" 8
        if [[ "${STATUS:-}" == "OK" ]]; then
            echo "Authorized - node is on the network."
            return 0
        fi
        echo "Still ${STATUS:-unknown} - authorize node ${node} in Central, then test again."
    done
}

# Direct / setup mode: network id on the command line.
if [[ -n "${1-}" ]]; then
    valid_id "$1" || { err "invalid network id '$1' (16 hex chars)"; exit 1; }
    switch_to "${1,,}"
    exit $?
fi

if (( SETUP_MODE )); then
    err "--setup requires a network id"
    exit 1
fi

# Interactive: enumerate joined networks, offer switch or new join.
mapfile -t IDS < <(joined_ids)
echo ""
echo "Joined ZeroTier networks:"
if (( ${#IDS[@]} == 0 )); then
    echo "  (none)"
else
    i=1
    for id in "${IDS[@]}"; do
        line=$(zt_json | awk -v n="$id" '$1==n')
        printf '  %d) %s  status=%s ip=%s\n' "$i" "$id" \
            "$(echo "$line" | awk '{print $2}')" "$(echo "$line" | awk '{print $3}')"
        i=$((i+1))
    done
fi
echo "  n) join a new network"
echo "  q) quit"
read -r -p "Select: " choice || choice="q"
case "${choice}" in
    q|Q) echo "quit"; exit 0 ;;
    n|N)
        read -r -p "Network id to join (16 hex chars): " nid || exit 1
        nid="${nid// /}"
        valid_id "$nid" || { err "invalid network id '$nid'"; exit 1; }
        switch_to "${nid,,}"
        ;;
    *)
        if [[ "$choice" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#IDS[@]} )); then
            switch_to "${IDS[$((choice-1))]}"
        else
            err "invalid selection '${choice}'"
            exit 1
        fi
        ;;
esac
