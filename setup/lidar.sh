# lidar.sh: RoboSense RSAIRY provisioning - UDP receive buffers + the dedicated Ethernet
# link (NetworkManager profiles, hot-plug dispatcher, live IP auto-detect).

# Fallback values; scripts/config_lidar.sh overwrites these from a live sniff
# when a LiDAR is reachable at setup time.
RSLIDAR_NIC="enP8p1s0"
RSLIDAR_HOST_IP="192.168.1.102"
RSLIDAR_LIDAR_IP="192.168.1.200"
RSLIDAR_DISPATCHER="/etc/NetworkManager/dispatcher.d/90-rslidar"
RSLIDAR_SYSCTL="/etc/sysctl.d/99-rslidar.conf"

# RSAIRY LiDAR sysctl (UDP receive buffer)
setup_lidar_sysctl() {
    step "RSAIRY LiDAR sysctl (UDP rmem)"

    sudo tee "${RSLIDAR_SYSCTL}" > /dev/null << 'EOF'
# UDP rmem ceiling for RSAIRY burst (~32 MB/s). Default ~200KB drops bursts.
net.core.rmem_max=26214400
net.core.rmem_default=26214400
EOF
    # `-p file` not `--system`: avoids reapplying unrelated L4T-incompatible drop-ins.
    sudo sysctl -p "${RSLIDAR_SYSCTL}" >/dev/null

    STEPS_RUN+=("lidar_sysctl")
    ok "sysctl: net.core.rmem_max=net.core.rmem_default=25 MiB"
}

# RSAIRY LiDAR network (NetworkManager + dispatcher)
setup_lidar_network() {
    step "RSAIRY LiDAR network (NetworkManager)"

    if is_provisioning_link "${RSLIDAR_NIC}"; then
        warn "setup is running over ${RSLIDAR_NIC} - deferring the LiDAR network configuration"
        warn "configuring it now drops this connection; join Wi-Fi first, then run 'config_lidar'"
        STEPS_DEFERRED+=("lidar_network")
        return
    fi

    for con_name in rslidar dev; do
        if nmcli -t -f NAME connection show 2>/dev/null | grep -qxF "$con_name"; then
            nmcli connection delete "$con_name" >/dev/null
        fi
    done

    # Sweep stray auto-profiles bound to the NIC (e.g. "Wired connection 1").
    while IFS=: read -r name dev; do
        [[ "$dev" == "${RSLIDAR_NIC}" ]] || continue
        case "$name" in
            rslidar|dev) ;;
            *) nmcli connection delete "$name" >/dev/null 2>&1 || true ;;
        esac
    done < <(nmcli -t -f NAME,DEVICE connection show)

    # priority 10 ⇒ NM activates this before DHCP fallback on link-up.
    # ipv4.routes is required: NM sets IFA_F_NOPREFIXROUTE on manual addresses,
    # so the kernel's connected-route is suppressed and LiDAR traffic would
    # otherwise route out wifi via the default gateway.
    nmcli connection add type ethernet con-name rslidar ifname "${RSLIDAR_NIC}" \
        ipv4.method manual \
        ipv4.addresses "${RSLIDAR_HOST_IP}/24" \
        ipv4.routes "192.168.1.0/24 0.0.0.0" \
        autoconnect yes \
        connection.autoconnect-priority 10 >/dev/null

    # DHCP fallback for when the dispatcher decides no LiDAR is present.
    nmcli connection add type ethernet con-name dev ifname "${RSLIDAR_NIC}" \
        ipv4.method auto \
        ipv4.dhcp-timeout 8 \
        autoconnect yes \
        connection.autoconnect-priority 0 >/dev/null

    # Dispatcher: ARP-probe the LiDAR on rslidar-up; fall back to DHCP after 8s.
    sudo tee "${RSLIDAR_DISPATCHER}" > /dev/null << EOL
#!/bin/bash
# Installed by setup.sh. Switches 'rslidar' static -> 'dev' DHCP on no-response.
IFACE="\$1"
ACTION="\$2"

[[ "\$IFACE" != "${RSLIDAR_NIC}" ]] && exit 0
[[ "\$ACTION" != "up" ]] && exit 0

ACTIVE=\$(nmcli -t -f NAME connection show --active 2>/dev/null | grep -xE 'rslidar|dev' | head -1)
[[ "\$ACTIVE" != "rslidar" ]] && exit 0

# 8s covers cold-boot LiDAR; -s pins ARP source IP (kernel otherwise picks wifi).
for _ in 1 2 3 4 5 6 7 8; do
    if arping -c 1 -w 1 -s ${RSLIDAR_HOST_IP} -I "\$IFACE" ${RSLIDAR_LIDAR_IP} >/dev/null 2>&1; then
        logger -t rslidar-net "LiDAR detected at ${RSLIDAR_LIDAR_IP}; staying on static."
        exit 0
    fi
done

logger -t rslidar-net "LiDAR not detected after 8 s; switching to DHCP."
nmcli connection down rslidar >/dev/null 2>&1 || true
nmcli connection up dev >/dev/null 2>&1 || true
EOL
    sudo chmod 755 "${RSLIDAR_DISPATCHER}"
    sudo chown root:root "${RSLIDAR_DISPATCHER}"

    STEPS_RUN+=("lidar_network")
    ok "NM: 'rslidar' (static ${RSLIDAR_HOST_IP}/24) + 'dev' (DHCP) + dispatcher installed (fallback values)"

    # Live sniff overwrites fallback values when a LiDAR is reachable.
    if ip link show "${RSLIDAR_NIC}" 2>/dev/null | grep -qE 'LOWER_UP'; then
        ok "carrier on ${RSLIDAR_NIC} is UP - running config_lidar to auto-detect actual LiDAR IPs"
        if sudo bash "${WORKSPACES}/scripts/config_lidar.sh"; then
            ok "config_lidar succeeded: rslidar configured against discovered LiDAR"
        else
            warn "config_lidar didn't detect a LiDAR. Fallback values remain active."
            warn "Once the LiDAR is plugged in and powered, run 'config_lidar' manually."
        fi
    else
        warn "no carrier on ${RSLIDAR_NIC} - skipping auto-detect."
        warn "After the LiDAR is plugged in and powered, run 'config_lidar' to auto-configure."
    fi
}
