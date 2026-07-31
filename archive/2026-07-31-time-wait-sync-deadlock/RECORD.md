# Archive record — 2026-07-31 — `systemd-time-wait-sync` apt deadlock

Archived record of the investigation and fix. The machine-applicable diff is
`time-wait-sync-guard.patch` in this directory.

**Root cause is fixed upstream**, in the ARK-OS fork: `indro-robotics/ARK-OS`, branch
`ARID_L4T_36.4.4`, commit `421b8bf` - `tools/install_software.sh` no longer enables the unit.
The `setup.sh` guard recorded here (`965647c`) is the second layer.

| | |
|---|---|
| Host | `arid` (Jetson, L4T / Linux 5.15.148-tegra) |
| Repo | `/home/jetson/workspaces` |
| Branch | `v1.3_rslidar` |
| Base commit | `20d5e73` — *fix(setup): gate the ROS2 question on the ros2 executable* |
| Files changed | `setup.sh`, `setup/system.sh` |

---

## Timeline

1. **Reported symptom.** `setup.sh` stalled with no output during ARK-OS `install.sh`, last
   lines being snapd postinst chatter about disabled/static units.

2. **First hypothesis — wrong.** Guessed snap *seeding* was stuck (`snap wait system
   seed.loaded`). Disproved immediately: `snapd.seeded.service` had completed in 62 ms.

3. **Actual cause found** via `systemctl list-jobs`: job 22,
   `systemd-time-wait-sync.service`, state `running`, with `time-sync.target`,
   `timers.target` and `graphical.target` all `waiting` behind it.

4. **Confirmed the mechanism** (see "Corrections" - the first version of this was wrong).
   The unit checks the clock **once at startup** via `adjtimex`. If the clock is not synced yet
   it falls back to an inotify wait on `/run/systemd/timesync/synchronized` - a file only
   `systemd-timesyncd` writes - and never re-checks `adjtimex`. `enable_clock_sync` masks
   timesyncd in favour of chrony, and chrony needs wifi then NTP (~20 s), so the unit always
   loses that race at boot. With `TimeoutStartSec=infinity` and `DefaultDependencies=no` it then
   waits forever in the earliest boot transaction.

   The trigger is specific: snapd's postinst runs `deb-systemd-invoke restart` over its units,
   which calls `systemctl restart` (no `--no-block`) on `snapd.snap-repair.timer`. That is a
   calendar timer (`OnCalendar=*-*-* 5,11,17,23:00`), and systemd implicitly orders every
   calendar timer `After=time-sync.target`. The target never opens, so systemctl never returns,
   so dpkg, apt and ARK-OS `install.sh` hang with no error output.

5. **Unblocked live.** `stop --no-block` + `mask` → pending jobs went **20 → 1**, unit
   `activating` → `inactive`. The stalled `install.sh` resumed unaided and proceeded to
   compiling Fast-DDS at `-j8`.

6. **Made preemptive.** Added `guard_time_wait_sync` and wired it into both setup entry paths.

7. **Verified no loss.** `time-sync.target` reached `active` at 00:44:25 *with the unit masked*,
   because `chrony.service` declares `Wants=`/`Before=time-sync.target` itself. Clock measured
   445 µs off NTP, stratum 3, all six peers at reach `377`.

## Why masking is a fix, not a workaround

The unit's function is to make `time-sync.target` mean "the clock is genuinely correct" rather
than "a time daemon started". It is built for boards with no RTC, where TLS, Kerberos and cron
would otherwise run against a 1970 clock.

This board has a working RTC (`rtc0`, `nvvrs-pseq`, maintained by chrony's `rtcsync`), and
nothing on it orders `After=time-sync.target` - not PX4, not uXRCE-DDS, not the ARID services.
The only consumers are `anacron`, `kerneloops` and two unused `isc-dhcp-server` units. Ubuntu
ships the unit **disabled** (`UnitFilePreset=disabled`); ARK-OS was the only thing enabling it.

Masking therefore restores the distro default on a board where ARK's timesyncd assumption does
not hold. `time-sync.target` is still reached - `chrony.service` declares `Wants=`/`Before=` it
directly. Verified active with the unit masked.

## Discarded theory, recorded so it isn't retried

> *"Maybe chrony should be installed last because its slew is too long."*

Reordering chrony does not help, but the reason is subtler than first recorded. The unit *does*
read the clock - once, at startup, via `adjtimex`. What it never does is re-check. Chrony's
install position cannot change the fact that at second zero of boot the clock is not yet synced,
because chrony needs wifi and then NTP regardless of when it was installed.

Proof the unit is not inherently broken on a chrony box: with the clock already synced, running
`/lib/systemd/systemd-time-wait-sync` directly exits **0 in 3 ms**, reporting
`adjtime state 0 status 0`. It is a lost startup race, not an impossibility.

## Interaction with ARK-OS

`ARK-OS/tools/install_software.sh:374` ran a bare
`sudo systemctl enable systemd-time-wait-sync.service`. **That line is now removed** in the fork
(`ARID_L4T_36.4.4`, `421b8bf`), replaced by a comment so an upstream merge cannot silently
reintroduce it. `setup/ark.sh` pins that repo and branch, so fresh installs pick the fix up.

On an ARK build that still has the line, the guard masks the unit first and ARK's enable then
fails non-fatally:

```
Failed to enable unit: Unit file /etc/systemd/system/systemd-time-wait-sync.service is masked.
```

That outcome is intended — it is ARK being prevented from re-arming the deadlock on every fresh
provision. ARK's `install.sh` completes normally afterwards. Nothing in ARK consumes
`time-sync.target`; the `enable` is an unconditioned gesture that assumes timesyncd.

## Verification evidence captured at the time

```
$ systemctl status systemd-time-wait-sync.service
   Active: activating (start) since Wed 2025-06-04 14:17:43 UTC; 1 year 1 month ago
   # NOTE: the "1 year 1 month" is FALSE - see "Corrections" below.

$ ls -la /run/systemd/timesync/
ls: cannot access '/run/systemd/timesync/': No such file or directory

$ systemctl is-enabled systemd-timesyncd
masked

$ systemctl is-active chronyd
active
```

After the fix:

```
$ systemctl is-active time-sync.target
active

$ chronyc tracking
Reference ID    : 179F10C2 (ntp.netlinkify.com)
Stratum         : 3
System time     : 0.000445282 seconds fast of NTP time

$ timedatectl
System clock synchronized: yes
       NTP service: active

$ sudo dpkg --audit
(empty — no broken or half-configured packages)

$ systemctl --failed
(empty)
```

## Corrections made during the investigation

**The "1 year 1 month" duration was wrong.** `systemctl status` reported the unit as activating
since 2025-06-04, and that was taken at face value. It is an artifact: this board's clock is
wrong during early boot and only corrected once chrony converges (~20 s in), so systemd records
unit start times against a bogus clock and the later step makes the delta look like months.
`uptime -s` showed the machine had been up ~2 hours at the time. Caught by the operator, not by
the investigation.

The diagnosis is unaffected - the unit hangs for the whole of whatever boot it is in, which is
ample to deadlock apt, and `TimeoutStartSec=infinity` is the real reason it never recovers.

**A `chrony-wait.service` replacement gate was built, tested, and then reverted.** The reasoning
was that masking the unit downgrades `time-sync.target` from "clock converged" to "chronyd
started". A bounded replacement was written and proven safe (4.0 s under an unsatisfiable sync
target, versus the original's infinite hang). It was reverted because measurement showed it
carries `Before=multi-user.target`, so it would have delayed `start_isaac_docker.service`,
`jetson-clocks.service` and `graphical.target` by ~10-13 s - and the operator confirmed PX4
alignment does not depend on absolute wall time. Cost without benefit.

## Testing limits

- The `stop --no-block` branch has **one** live observation (the real jam, 20 → 1 jobs).
  Reproducing a genuine jam needs a reboot with the unit unmasked.
- The mask and skip branches were exercised repeatedly and behaved correctly.
- Two early test runs reported `[WARN] could not mask`; that was a sandbox artifact — `sudo`'s
  cached credential does not cross into a child `bash -c` there — not a code defect. Confirmed
  by running the same function in the top-level shell, where it succeeded.

## Out of scope / not a repo change

`.claude/settings.local.json` was collapsed from a dozen accumulated one-off permission rules
into blanket tool allows. It is gitignored (via `/home/jetson/.config/git/ignore`) and is local
agent configuration, unrelated to this fix. Noted only so the working-tree state is explained.
