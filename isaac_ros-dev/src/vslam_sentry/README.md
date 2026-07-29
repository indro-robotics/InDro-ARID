# vslam_sentry

The sentry is the device-plane watchdog for the VSLAM stack. It watches the `camera_info` rate of
the front D435 infra1 and infra2 streams alongside VO cadence, classifies which layer broke, and
hardware-resets a wedged camera.

It never restarts VSLAM, never gates launch, and never touches fusion. In steady state it touches
the USB bus only to issue a reset.

## Camera states

| State | Meaning |
|---|---|
| `SETTLING` | Bringup is still in progress; no verdicts yet. |
| `HEALTHY` | Slowest stream at or above `min_hz`. |
| `DEGRADED` | Below rate for `degraded_ticks` windows. |
| `STREAM_DEAD` | Near 0 Hz for `dead_ticks` windows. |
| `GONE` | Serial not on the bus at reset time; a reset cannot help. |
| `RECOVERING` | Reset issued, inside the verify window. |
| `ESCALATED` | `max_reset_attempts` exhausted; sticky until real recovery. |

## VSLAM states

| State | Meaning |
|---|---|
| `SETTLING` | Pre-settle. |
| `OK` | VO at or above `vo_min_hz` with gaps within `vo_max_gap_ms`. |
| `STARVED` | Below rate or over gap. |
| `DOWN` | Near 0 Hz VO. |

## Topics and services

| Name | Type | Direction | Note |
|---|---|---|---|
| `/<cam>/{infra1,infra2}/camera_info` | `CameraInfo` | sub | Presence and rate oracle. |
| `/visual_slam/tracking/odometry` | `Odometry` | sub | VO cadence. |
| `/vslam_sentry/status` | `String` | pub | Latched JSON. |
| `/vslam_sentry/healthy` | `Bool` | pub | Latched aggregate. |
| `/vslam_sentry/status_now` | `Trigger` | srv | JSON on demand. |
| `/vslam_sentry/reset_<cam>` | `Trigger` | srv | Manual reset. |
| `/<cam>/hw_reset` | `Trigger` | cli | Driver reset, the primary path. |

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `config_path` | `""` | Path to `vslam_config.yaml` for serials and namespaces. |
| `settle_s` | `40.0` | Bringup grace before any verdict. |
| `tick_s` | `3.0` | Window period. |
| `min_hz` | `30.0` | Per-stream healthy floor. |
| `dead_ticks` | `2` | Near-0 Hz windows before `STREAM_DEAD`. |
| `degraded_ticks` | `3` | Low windows before `DEGRADED`. |
| `vo_min_hz` | `20.0` | VO floor for `OK`. |
| `vo_max_gap_ms` | `500.0` | Max VO inter-message gap. |
| `auto_reset` | `true` | Enable automatic recovery. |
| `reset_cooldown_s` | `75.0` | Minimum spacing between attempts on one camera. |
| `max_reset_attempts` | `3` | Attempts before `ESCALATED`. |
| `reset_verify_s` | `60.0` | Recovery verify window. |
| `reset_dead_time_s` | `8.0` | Window ignored immediately after a reset. |
| `post_reset_quarantine_s` | `30.0` | Bus-wide spacing between any two resets. |
| `reset_calm_ticks` | `5` | Consecutive calm VO windows required before an automatic reset. |
| `reset_defer_max_s` | `120.0` | Deferral ceiling; `0` disables deferral. |
| `escalated_retry_s` | `600.0` | Re-arm one attempt after this long; `0` stays sticky. |
| `status_period_s` | `15.0` | Latched status refresh. |

The launch sets `reset_defer_max_s` to `0`, so on this drone an automatic reset is never deferred.

## Reset policy

- One camera at a time; concurrent resets are never issued.
- The primary path is the driver `hw_reset` service, with a direct rs2 fallback only when that
  service is absent.
- `reset_cooldown_s` spaces attempts on a camera, and `max_reset_attempts` sends it to a sticky
  `ESCALATED`.
- `reset_dead_time_s` passes before the `reset_verify_s` verify window opens; streams back at rate
  are the proof of recovery.
- `post_reset_quarantine_s` spaces any two attempts bus-wide.
- A single targeted reset is airborne-safe, and positive stream evidence clears any bad state.

## Log

The node writes `$ISAAC_ROS_WS/run_logs/sentry/sentry.log`, falling back to
`/workspaces/isaac_ros-dev/run_logs/sentry/sentry.log`, and mirrors to the journal. A blank serial
in the config produces `NO CAMERAS parsed` and the node idles.

## Disable

Launching with `auto_reset:=false` keeps monitoring and status reporting while suppressing every
reset.
