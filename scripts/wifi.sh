#!/bin/bash
# wifi.sh - interactive Wi-Fi connection via NetworkManager (alias: wifi).
# Non-interactive: set ARID_WIFI_SSID (+ ARID_WIFI_PASS) to skip the picker.

set -uo pipefail

# An existing profile is modified in place: delete + re-add drops any session active on it.
create_profile() {
    local ssid="$1" pass="$2" sec="$3" hidden="${4:-no}"
    local key_mgmt="wpa-psk"
    [[ "${sec}" == *"SAE"* ]] && key_mgmt="sae"

    local out

    # `-e no` disables nmcli's literal-`:` escaping, without which an SSID containing `:` is
    # listed back escaped and never matches the profile name.
    if nmcli -e no -t -f NAME con show | grep -Fqx "${ssid}"; then
        if [[ -n "${pass}" ]]; then
            if ! out=$(sudo nmcli con modify "${ssid}" \
                wifi-sec.key-mgmt "${key_mgmt}" \
                wifi-sec.psk "${pass}" \
                802-11-wireless.hidden "${hidden}" \
                connection.autoconnect yes \
                connection.autoconnect-retries 3 2>&1); then
                echo "Failed to update profile '${ssid}':" >&2
                echo "${out}" | sed 's/^/  /' >&2
                return 1
            fi
        else
            if ! out=$(sudo nmcli con modify "${ssid}" \
                wifi-sec.key-mgmt "" \
                wifi-sec.psk "" \
                802-11-wireless.hidden "${hidden}" \
                connection.autoconnect yes \
                connection.autoconnect-retries 3 2>&1); then
                echo "Failed to update profile '${ssid}':" >&2
                echo "${out}" | sed 's/^/  /' >&2
                return 1
            fi
        fi
    else
        if ! out=$(sudo nmcli con add type wifi ifname '*' \
            con-name "${ssid}" autoconnect yes ssid "${ssid}" 2>&1); then
            echo "Failed to create profile '${ssid}':" >&2
            echo "${out}" | sed 's/^/  /' >&2
            return 1
        fi
        if ! out=$(sudo nmcli con modify "${ssid}" \
            802-11-wireless.hidden "${hidden}" \
            connection.autoconnect-retries 3 2>&1); then
            echo "Failed to set hidden/retries for '${ssid}':" >&2
            echo "${out}" | sed 's/^/  /' >&2
            sudo nmcli con delete "${ssid}" >/dev/null 2>&1
            return 1
        fi
        if [[ -n "${pass}" ]]; then
            if ! out=$(sudo nmcli con modify "${ssid}" \
                wifi-sec.key-mgmt "${key_mgmt}" \
                wifi-sec.psk "${pass}" 2>&1); then
                echo "Failed to set password for '${ssid}':" >&2
                echo "${out}" | sed 's/^/  /' >&2
                sudo nmcli con delete "${ssid}" >/dev/null 2>&1
                return 1
            fi
        fi
    fi

    echo "Profile '${ssid}' saved. Hotspot disconnecting; attempting to join '${ssid}'; falls back to hotspot on failure."
}

if [[ -n "${ARID_WIFI_SSID:-}" ]]; then
    sec=""
    [[ -n "${ARID_WIFI_PASS:-}" ]] && sec="WPA2"
    create_profile "${ARID_WIFI_SSID}" "${ARID_WIFI_PASS:-}" "${sec}" no || exit 1
    exit 0
fi

sudo nmcli radio wifi on

# `-e no` disables nmcli's literal-':' escaping, so an SSID containing ':' arrives split across
# fields 3..NF rather than escaped; the awk below rejoins it.
mapfile -t NETS < <(
    nmcli -e no -t -f SIGNAL,SECURITY,SSID device wifi list --rescan yes 2>/dev/null \
        | awk -F: '{
            sig = $1
            sec = $2
            ssid = $3
            for (i = 4; i <= NF; i++) ssid = ssid ":" $i
            if (ssid != "" && ssid != "--") print ssid "\t" sig "\t" sec
        }' \
        | sort -t$'\t' -k2,2 -n -r \
        | awk -F'\t' '!seen[$1]++'
)

if (( ${#NETS[@]} == 0 )); then
    echo "No networks visible." >&2
    exit 1
fi

printf "\n  %-3s  %-32s  %-7s  %s\n" "#"  "SSID" "Signal" "Security"
printf   "  %-3s  %-32s  %-7s  %s\n" "-"  "----" "------" "--------"
for i in "${!NETS[@]}"; do
    IFS=$'\t' read -r ssid signal security <<< "${NETS[$i]}"
    [[ -z "${security}" || "${security}" == "--" ]] && security="open"
    printf "  %-3d  %-32s  %-7s  %s\n" "$((i+1))" "${ssid:0:32}" "${signal}" "${security}"
done

echo ""
read -r -p "# (Enter = manual SSID): " choice

HIDDEN=no
if [[ -z "${choice}" ]]; then
    read -r -p "SSID: " SSID || SSID=""
    [[ -z "${SSID}" ]] && { echo "Aborted." >&2; exit 1; }
    PASS=""
    read -r -s -p "Password for '${SSID}' (Enter = open): " PASS || PASS=""
    echo
    SECURITY="WPA2"   # A typed SSID has no scan entry, so security is assumed; unused when PASS is empty.
    HIDDEN=yes        # A typed SSID is the hidden-network path: NM must actively probe for it.
elif [[ "${choice}" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#NETS[@]} )); then
    idx=$((choice - 1))
    IFS=$'\t' read -r SSID _ SECURITY <<< "${NETS[$idx]}"
    PASS=""
    if [[ -z "${SECURITY}" || "${SECURITY}" == "--" ]]; then
        PASS=""
    else
        read -r -s -p "Password for '${SSID}': " PASS
        echo
    fi
else
    echo "Invalid #." >&2
    exit 1
fi

create_profile "${SSID}" "${PASS}" "${SECURITY}" "${HIDDEN}" || exit 1
