# Archive record — 2026-07-31 — LiDAR dispatcher ARP self-detection

Field bring-up of `arid`. The headline defect is a self-referential LiDAR auto-detection that made
the LiDAR link tear itself down on every link-up. Three smaller fixes found in the same session are
recorded at the end.

| | |
|---|---|
| Host | `arid` (Jetson Orin NX Super, L4T R36.4.3) |
| Repo | `/home/jetson/workspaces`, branch `v1.3_rslidar` |
| Fix commit | `1537a80` — *fix(lidar): never accept this host as the detected LiDAR* |

---

## 1. Symptom

`/rslidar_points` silent, `/rslidar_coordinator/alive` false, SDK logging `ERRCODE_MSOPTIMEOUT`
continuously. Desktop showed repeated *"Activation of network connection failed"* popups.
`lidar_diag` reported no IPv4 and no route on `enP8p1s0`, yet its passive sniff saw the LiDAR
transmitting perfectly:

```
192.168.1.200.49609 > 192.168.1.102.6699: UDP, length 1248
```

## 2. Root cause

`config_lidar.sh` sniffs the NIC promiscuously and unfiltered by direction, so the capture contains
**this host's own ARP probes** alongside the LiDAR's traffic. The fallback extractors took the first
matching line with `head -1`. On a quiet link that line is frequently self-originated:

```
who-has 192.168.1.102 tell 192.168.1.102
```

which yields `LIDAR_IP` = the host's own address and `LIDAR_MAC` = the host's own NIC:

```
RSLIDAR_LIDAR_IP="192.168.1.102"       # the host
RSLIDAR_LIDAR_MAC="4c:bb:47:f7:83:d0"  # the host's NIC
```

Those values were then baked into `/etc/NetworkManager/dispatcher.d/90-rslidar`:

```bash
arping -c 1 -w 1 -s 192.168.1.102 -I "$IFACE" 192.168.1.102
```

The host ARPing **itself** to decide whether the LiDAR is present. It never answers, so after 8 tries
the dispatcher ran `nmcli connection down rslidar` and fell back to DHCP — on every link-up. Setup
had generated a watchdog that guaranteed the LiDAR link would tear itself down.

## 3. The failure chain

```
config_lidar matches its own ARP probe
  └─ rslidar_detected.conf: LiDAR IP == host IP
      └─ 90-rslidar dispatcher arpings the host's own address
          └─ no reply after 8s -> nmcli connection down rslidar
              └─ 'dev' DHCP profile autoconnects, times out (no DHCP server), retries forever
                  └─ enP8p1s0 ends with NO IPv4 address
                      └─ kernel drops the LiDAR's frames as not-for-us
                          └─ SDK socket on 0.0.0.0:6699 receives nothing
                              └─ ERRCODE_MSOPTIMEOUT, alive=false, no point cloud
```

## 4. Diagnostic dead ends, recorded so they are not retried

Frames were visible to `tcpdump` but never reached a bound UDP socket, which sent the
investigation through several wrong turns before the NM log gave it away:

- **Destination MAC mismatch** — disproved. `tcpdump -p` (non-promiscuous) still showed the frames,
  so they *were* addressed to this NIC.
- **Firewall / XDP / rp_filter** — all clear. `iptables -P INPUT ACCEPT` with no DROP rules, empty
  `raw`/`mangle`/`nft` rulesets, no ingress eBPF, `rp_filter=2` (loose).
- **A second process holding the socket** — only one `rslidar_sdk_node`, `Recv-Q 0`.

The decisive test was binding port 6699 with a plain Python socket: **zero packets in 4 s** while
`tcpdump` saw them arriving. `tcpdump` taps at the device layer, so the packets were being dropped
before local delivery — because the host held no `192.168.1.102` at all.

The giveaway was in the NetworkManager journal:

```
state change: activated -> deactivating (reason 'user-requested')
audit: op="connection-deactivate" name="rslidar" pid=78536 uid=0
```

`user-requested`, by root, exactly 8 s after each activation — the dispatcher's arping loop.

## 5. Fix

`candidate_is_self()` in `scripts/config_lidar.sh` rejects a candidate whose MAC is the local NIC,
whose LiDAR IP is an address this host holds, or where LiDAR and host IPs are equal. Both extractors
now walk **every** matching capture line instead of taking the first, so a self-match is skipped
rather than promoted. A final guard refuses to write the conf or the dispatcher at all if the
surviving candidate is still self-referential.

Verified against the exact failure plus three variants: rejects all self-matches, accepts only
`mac=08:48:57:14:f9:fa lidar=192.168.1.200 host=192.168.1.102`.

`setup/lidar.sh` needed no change — its fallbacks (host `.102`, LiDAR `.200`) are correct and it does
not source the detected conf, so a fresh install was never at risk. Only a `config_lidar` run was.

## 6. Result

```
enP8p1s0        192.168.1.102/24, stable
ping .200       0% loss, 0.245 ms
LiDAR MAC       08:48:57:14:f9:fa
/rslidar_points 10.006 Hz          (RSAIRY nominal ~10 Hz)
alive           data: true
```

---

## Other fixes from the same session

**Camera watchdog false alarm** (`pipelines.yaml`, `alive_threshold` 2.0 → 5.0). The value doubles
as the startup grace period, and a cold Argus open on the IMX477 takes ~2.04 s to first frame — so
it tripped on every single launch (`stalled ... frames resumed` one line later) and made
`verify_cv_cams` abandon a working camera. At 15 fps a real stall is still caught within 5 s.

The camera was never faulty. Two wrong diagnoses were made first and are worth recording: the sensor
was believed absent because the search was for an **IMX219 at 0x10** (the DT names three unfitted
IMX219 slots), when the fitted camera is an **IMX477 at 0x1a on bus 9** — present, bound, and
enumerated as `video0: vi-output, imx477 9-001a`. A `0x1a` seen in an early `i2cdetect` was
dismissed as an audio codec. Second, verification was believed to be fighting the camera manager
for `sensor-id=0`; it is not — the manager registers the pipeline at boot but does not start it, so
the sensor is free.

**Alias QoS hang** (`setup/system.sh`). `cam_down_alive` and `rslidar_alive` hung forever with no
error. `ros2 topic echo` uses its full default QoS profile only when passed **no** `--qos-*` flags;
once durability is overridden, reliability silently falls back to `BEST_EFFORT`, which cannot match
these `RELIABLE` publishers. Both aliases now pass `--qos-reliability reliable` alongside
`--qos-durability transient_local`. Both publishers were confirmed `RELIABLE` + `TRANSIENT_LOCAL`
before applying the same change to each — a `reliable` subscriber against a `best_effort` publisher
would fail in the opposite direction.

**Smoke test scored an absent LiDAR as passing** (`scripts/local_test.sh`, commit `e10122f`). 5e
treated the watchdog reporting `alive == false` as a pass, and 5f skipped on zero cloud messages, so
a drone with the LiDAR unplugged reported 42 passed / 1 skipped. Both are now FAIL, matching
`cam_down_alive` which already failed on the identical condition.

**NoMachine** (`setup/system.sh`, commit `cba0e01`). Their ARM download page now serves only
`nomachine-personal-edition`, which installs cleanly and then refuses every connection with *"the
subscription license on this server has expired"*. The step now installs a vendored `.deb` from
`local_ws/auxiliary/nomachine/` and rejects any `*personal-edition*` package. Separately,
`.Xauthority` is removed with `rm -rf` rather than `rm -f`: docker creates it as a *directory* when
a container bind-mounts a non-existent host path, and the failing `rm` aborted the whole
provisioning run under `set -euo pipefail`.

## Testing limits

- The dispatcher fix is verified by unit-testing `candidate_is_self` against captured values, not by
  re-running `config_lidar` end to end on hardware. The live dispatcher and detected conf on this
  host were hand-corrected; a real `config_lidar` run would regenerate both and is the outstanding
  end-to-end proof.
- `setup_bashrc` was observed printing `[OK] .bashrc updated` after a token substitution failed,
  leaving `alias foxglove_bridge=''`. Its `@@TOKEN@@` leftover guard cannot catch this: the token
  *was* replaced, with an empty string. Not fixed - flagged only.
