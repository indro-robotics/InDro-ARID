# vslam_sentry

The sentry is the device-plane watchdog for the VSLAM stack. It measures the `camera_info` rate of each D435 IR stream and the VO cadence, classifies which layer has broken, and issues a targeted hardware reset to a wedged camera one at a time.

It never restarts VSLAM, never gates launch and never touches fusion. The bus is touched only at a reset, because in steady state the streams themselves are the presence oracle.

## Camera states

Each camera holds one of seven states.

| State | Meaning |
|---|---|
| `SETTLING` | The bringup grace window is open, so no verdict is issued. |
| `HEALTHY` | The slower stream is at or above `min_hz`. |
| `DEGRADED` | Below rate for `degraded_ticks` windows. |
| `STREAM_DEAD` | Near zero Hz for `dead_ticks` windows. |
| `GONE` | The serial was not on the bus at reset time, so a reset cannot help. |
| `RECOVERING` | A reset was issued and the verify window is open. |
| `ESCALATED` | `max_reset_attempts` exhausted; sticky until real recovery. |

## VSLAM states

VO cadence is classified into four states.

| State | Meaning |
|---|---|
| `SETTLING` | Pre-settle. |
| `OK` | VO at or above `vo_min_hz` with gaps within `vo_max_gap_ms`. |
| `STARVED` | Below rate or over gap. |
| `DOWN` | Near zero Hz VO. |

## Topics and services

The sentry reads stream and VO rates, and publishes its verdicts.

| Name | Type | Direction | Note |
|---|---|---|---|
| `/<cam>/{infra1,infra2}/camera_info` | `sensor_msgs/CameraInfo` | sub | Presence and rate oracle. |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | sub | VO cadence. |
| `/vslam_sentry/status` | `std_msgs/String` | pub | Latched status JSON. |
| `/vslam_sentry/healthy` | `std_msgs/Bool` | pub | Latched; true only when settled, VO `OK` and every camera `HEALTHY`. |
| `/vslam_sentry/status_now` | `std_srvs/Trigger` | srv | The same JSON on demand. |
| `/vslam_sentry/reset_<cam>` | `std_srvs/Trigger` | srv | Manual per-camera reset. |
| `/<cam>/hw_reset` | `std_srvs/Trigger` | cli | Driver reset, the primary path. |

## Parameters

All parameters are declared on the node with the defaults below.

| Name | Default | Meaning |
|---|---|---|
| `config_path` | `""` | Path to `vslam_config.yaml` for serials and namespaces. |
| `settle_s` | `40.0` | Bringup grace before any verdict. |
| `tick_s` | `3.0` | Measurement window period. |
| `min_hz` | `30.0` | Per-stream healthy floor. |
| `dead_ticks` | `2` | Near-zero windows before `STREAM_DEAD`. |
| `degraded_ticks` | `3` | Low windows before `DEGRADED`. |
| `vo_min_hz` | `20.0` | VO `OK` floor. |
| `vo_max_gap_ms` | `500.0` | Maximum VO inter-message gap. |
| `auto_reset` | `true` | Enables automatic recovery. |
| `reset_cooldown_s` | `75.0` | Minimum spacing between one camera's attempts. |
| `max_reset_attempts` | `3` | Attempts before `ESCALATED`. |
| `reset_verify_s` | `60.0` | Recovery verify window. |
| `reset_dead_time_s` | `8.0` | Straddling window ignored after a reset. |
| `post_reset_quarantine_s` | `30.0` | Bus-wide spacing between any two resets. |
| `reset_calm_ticks` | `5` | Consecutive VO-OK windows required before an automatic reset. |
| `reset_defer_max_s` | `120.0` | Deferral ceiling; `0` disables deferral. |
| `escalated_retry_s` | `600.0` | Re-arm interval for `ESCALATED`; `0` keeps it sticky. |
| `status_period_s` | `15.0` | Latched status refresh period. |

## Reset policy

Resets are rate-limited and serialized across the bus.

- One camera at a time, never concurrent.
- The driver `hw_reset` service is the primary path; a direct rs2 reset is the fallback when that service is absent.
- Each camera has a `reset_cooldown_s` cooldown, and `max_reset_attempts` failures leave it `ESCALATED`.
- After `reset_dead_time_s` the verify window runs for `reset_verify_s`, and streams back at rate are the proof of recovery.
- `post_reset_quarantine_s` separates any two attempts bus-wide.
- Automatic resets are deferred until VO has been calm for `reset_calm_ticks`, up to `reset_defer_max_s`, and fire early if a second camera is unhealthy.
- A single targeted reset is safe airborne, and positive stream evidence clears any bad state.

Resets are held entirely while all streams and VO are dead in the same window. That pattern is a frozen driver process or a bus-wide power loss rather than one camera wedging, and a driver reset cannot address either; the log records `PROCESS-PLANE FAULT suspected`. Any single stream recovering clears the hold. Recovery from that condition is a supervisor VSLAM cycle.

## Log

The sentry writes to `$ISAAC_ROS_WS/run_logs/sentry/sentry.log` and to the journal.

## Disable

The sentry can be reduced to monitoring or removed entirely.

- `auto_reset:=false` keeps monitoring and status but issues no resets.
- Removing `vslam_sentry_node` from `px4_vslam/launch/vslam.launch.py` turns it off entirely.
