# network.sh: off-board networking (ZeroTier). The LiDAR link lives in setup/lidar.sh.

# ZeroTier daemon + optional join (zt_join.sh, single-network model). ACCESS_DENIED is not
# a failure: the node works the moment it is authorized in ZeroTier Central.
setup_zerotier() {
    step "ZeroTier"

    if ! command -v zerotier-cli >/dev/null 2>&1; then
        curl -s https://install.zerotier.com | sudo bash || true
        if ! command -v zerotier-cli >/dev/null 2>&1; then
            err "ZeroTier install failed (installer needs internet)"
            return 1
        fi
        ok "ZeroTier installed"
    fi
    sudo systemctl enable --now zerotier-one >/dev/null 2>&1 || true

    # Already a member of a network: leave it unchanged (switching is zt_join's job).
    local joined
    joined=$(sudo zerotier-cli -j listnetworks 2>/dev/null \
        | python3 -c "import json,sys;print(' '.join(n['nwid'] for n in json.load(sys.stdin)))" 2>/dev/null || true)
    if [[ -n "${joined// /}" ]]; then
        skip "already on ZeroTier network(s): ${joined} - use 'zt_join' to switch"
        STEPS_RUN+=("zerotier")
        return
    fi

    local net=""
    if (( PRE )); then
        net="${PRE_ZTNET:-}"
    else
        read -r -p "  ZeroTier network id to join (16 hex chars, Enter = skip): " net || net=""
    fi
    net="${net// /}"
    if [[ -z "${net}" || "${net}" == "skip" ]]; then
        skip "no ZeroTier network joined - run 'zt_join' later if needed"
        STEPS_RUN+=("zerotier")
        return
    fi
    if bash "${WORKSPACES}/scripts/zt_join.sh" --setup "${net}"; then
        ok "ZeroTier network ${net} joined"
    else
        warn "ZeroTier join incomplete (authorize this node in ZeroTier Central, or run 'zt_join' later)"
    fi
    STEPS_RUN+=("zerotier")
}
