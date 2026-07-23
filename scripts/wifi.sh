#!/bin/bash
# wifi.sh - interactive Wi-Fi connection via NetworkManager (alias: wifi).
# Profiles go through `nmcli con add` / `con modify`; safe from the drone's own hotspot
# (autoconnect priority handles the AP-to-STA transition).
# Non-interactive: set ARID_WIFI_SSID (+ ARID_WIFI_PASS) to skip the picker.

set -uo pipefail

# create_profile <ssid> <pass> <security> <hidden>
# Creates a new NM profile, or modifies an existing one of the same name (so an active
# session on the same profile is not destroyed by a delete + re-add).
create_profile() {
    local ssid="$1" pass="$2" sec="$3" hidden="${4:-no}"
    local key_mgmt="wpa-psk"
    [[ "${sec}" == *"SAE"* ]] && key_mgmt="sae"

    local out

    # `-e no` disables nmcli's literal-`:` escaping so SSIDs with embedded `:` match the
    # profile-name list correctly (otherwise `MyNet:5G` stored as `MyNet\:5G` never matches).
    if nmcli -e no -t -f NAME con show | grep -Fqx "${ssid}"; then
        # Existing profile: modify in place to avoid disrupting an active connection on it.
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
            # Open network: clear any prior security.
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
        # New profile.
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

# Non-interactive path.
if [[ -n "${ARID_WIFI_SSID:-}" ]]; then
    sec=""
    [[ -n "${ARID_WIFI_PASS:-}" ]] && sec="WPA2"
    create_profile "${ARID_WIFI_SSID}" "${ARID_WIFI_PASS:-}" "${sec}" no || exit 1
    exit 0
fi

# Interactive path.
sudo nmcli radio wifi on

# `--rescan yes` forces a fresh scan and waits for completion; no separate rescan + sleep.
# `-e no` disables nmcli's literal-':' escaping so SSIDs with embedded ':' parse unambiguously.
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
    SECURITY="WPA2"   # Assume WPA2 when secured; key-mgmt branch ignored for open (empty PASS).
    HIDDEN=yes        # Manual SSID is the hidden-network path; NM must probe for it.
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
