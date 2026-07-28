# vslam_sentry

Device-plane watchdog for the VSLAM stack. Watches `camera_info` rate of the 3 D435s (infra1+infra2) and VO cadence, classifies which layer broke, hardware-resets a wedged camera one at a time.

Never restarts VSLAM, never gates launch, never touches fusion. Bus touched only at reset (streams are the presence oracle in steady state).

## Per-camera states

| State | Meaning |
|---|---|
| `SETTLING` | bringup owns cameras; no verdicts yet |
| `HEALTHY` | min stream ≥ `min_hz` |
| `DEGRADED` | below rate for `degraded_ticks` windows |
| `STREAM_DEAD` | ~0 Hz for `dead_ticks` windows |
| `GONE` | serial not on bus at reset — harness/contact class, reset can't help |
| `RECOVERING` | reset issued, in verify window |
| `ESCALATED` | `max_reset_attempts` exhausted; sticky until real recovery |

## VSLAM states

| State | Meaning |
|---|---|
| `SETTLING` | pre-settle |
| `OK` | vo ≥ `vo_min_hz`, gap ≤ `vo_max_gap_ms` |
| `STARVED` | below rate or over gap |
| `DOWN` | ~0 Hz VO |

## Topics / services

| Name | Type | Dir | Note |
|---|---|---|---|
| `/<cam>/{infra1,infra2}/camera_info` | CameraInfo | sub | presence/rate oracle |
| `/visual_slam/tracking/odometry` | Odometry | sub | VO cadence |
| `/vslam_sentry/status` | String | pub | latched JSON |
| `/vslam_sentry/healthy` | Bool | pub | latched aggregate |
| `/vslam_sentry/status_now` | Trigger | srv | JSON on demand |
| `/vslam_sentry/reset_<cam>` | Trigger | srv | manual reset |
| `/<cam>/hw_reset` | Trigger | cli | driver reset (primary path) |

## Params

| Name | Default | Meaning |
|---|---|---|
| `config_path` | `""` | vslam_config.yaml — serials/namespaces |
| `settle_s` | `40.0` | bringup grace before any verdict |
| `tick_s` | `3.0` | window period |
| `min_hz` | `30.0` | per-stream healthy floor |
| `dead_ticks` | `2` | ~0 Hz windows → STREAM_DEAD |
| `degraded_ticks` | `3` | low windows → DEGRADED |
| `vo_min_hz` | `20.0` | VO OK floor |
| `vo_max_gap_ms` | `500.0` | max VO inter-msg gap |
| `auto_reset` | `true` | enable auto recovery |
| `reset_cooldown_s` | `75.0` | min spacing between a cam's attempts |
| `max_reset_attempts` | `3` | attempts before ESCALATED |
| `reset_verify_s` | `60.0` | recovery verify window |
| `reset_dead_time_s` | `8.0` | ignore straddling window post-reset |
| `post_reset_quarantine_s` | `30.0` | bus-wide spacing between ANY two resets |
| `reset_calm_ticks` | `5` | consecutive sane V_OK windows required before an AUTO reset fires (defer out of starvation) |
| `reset_defer_max_s` | `120.0` | defer ceiling; also fires early if a 2nd camera is unhealthy; `0` = no deferral |
| `escalated_retry_s` | `600.0` | ESCALATED re-arm: one fresh attempt after this long; `0` = sticky forever |
| `status_period_s` | `15.0` | latched status refresh |

## Reset policy

- One camera at a time — `active_reset_` token, never concurrent.
- Primary path = driver `hw_reset` service; direct rs2 fallback only if driver service absent.
- Cooldown `reset_cooldown_s` per camera; `max_reset_attempts` then sticky `ESCALATED`.
- Dead-time `reset_dead_time_s` before verify; verify window `reset_verify_s`; streams at rate = proof.
- Bus-wide quarantine `post_reset_quarantine_s` between any two attempts.
- Airborne-safe: singular targeted reset allowed; positive stream evidence clears any bad state.

## Log

`$ISAAC_ROS_WS/run_logs/sentry/sentry.log` (fallback `/workspaces/isaac_ros-dev/...`); also journal.

## Disable

- `auto_reset:=false` — keep monitoring/status, no resets.
- Remove `vslam_sentry_node` from `px4_vslam/launch/vslam.launch.py` — off entirely.
