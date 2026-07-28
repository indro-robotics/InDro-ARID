// vslam_sentry: device-plane watchdog for the VSLAM stack.
//
// Watches per-camera stream rates (camera_info, never image topics) and VO output
// cadence, classifies which layer broke (camera fw / USB / driver / cuVSLAM), and
// recovers a wedged camera with a targeted hardware_reset() by serial - one camera
// at a time, never concurrent. Reads the same vslam_config.yaml the driver launch
// loads, so serials have a single source of truth.
//
// librealsense discipline: NO persistent rs2::context, no hotplug callback, no
// periodic enumeration. A second librealsense client probing a live bus is the
// proven cross-probe kill vector on this stack, so the bus is touched exactly once
// per reset attempt (scoped context inside issue_reset), and only for a camera that
// is already classified dead - the streams are the presence oracle in steady state.
//
// Role boundaries: arid_supervisor owns the process plane (bringup/teardown), the
// reactor owns the data plane (VO gating to EKF2). The sentry never restarts VSLAM,
// never gates launch, never touches fusion.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <librealsense2/rs.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

std::string wall_stamp() {
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count() % 1000;
  char buf[32];
  std::strftime(buf, sizeof(buf), "%d/%m %H:%M:%S", std::localtime(&t));
  char out[48];
  std::snprintf(out, sizeof(out), "%s,%03d", buf, static_cast<int>(ms));
  return out;
}

double steady_now() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

const char *state_name(int s) {
  switch (s) {
    case 0: return "SETTLING";
    case 1: return "HEALTHY";
    case 2: return "DEGRADED";
    case 3: return "STREAM_DEAD";
    case 4: return "GONE";
    case 5: return "RECOVERING";
    case 6: return "ESCALATED";
  }
  return "UNKNOWN";
}

}  // namespace

class VslamSentry : public rclcpp::Node {
 public:
  enum CamState { SETTLING = 0, HEALTHY, DEGRADED, STREAM_DEAD, GONE, RECOVERING, ESCALATED };
  enum VslamState { V_SETTLING = 0, V_OK, V_STARVED, V_DOWN };

  VslamSentry() : Node("vslam_sentry") {
    config_path_   = declare_parameter<std::string>("config_path", "");
    settle_s_      = declare_parameter<double>("settle_s", 40.0);
    tick_s_        = declare_parameter<double>("tick_s", 3.0);
    min_hz_        = declare_parameter<double>("min_hz", 30.0);
    dead_ticks_    = declare_parameter<int>("dead_ticks", 2);
    degraded_ticks_= declare_parameter<int>("degraded_ticks", 3);
    vo_min_hz_     = declare_parameter<double>("vo_min_hz", 20.0);
    vo_max_gap_ms_ = declare_parameter<double>("vo_max_gap_ms", 500.0);
    auto_reset_    = declare_parameter<bool>("auto_reset", true);
    cooldown_s_    = declare_parameter<double>("reset_cooldown_s", 75.0);
    max_attempts_  = declare_parameter<int>("max_reset_attempts", 3);
    verify_s_      = declare_parameter<double>("reset_verify_s", 60.0);
    reset_dead_time_s_ = declare_parameter<double>("reset_dead_time_s", 8.0);
    quarantine_s_  = declare_parameter<double>("post_reset_quarantine_s", 30.0);
    // Calm-window deferral: an auto reset (re-enumeration = the expensive, contended
    // operation) only fires after calm_ticks consecutive sane V_OK windows, so it
    // lands in the cheap ~100ms-class regime instead of stacking on top of an
    // existing VO starvation. Bounded by defer_max_s (0 = defer disabled).
    calm_ticks_    = declare_parameter<int>("reset_calm_ticks", 5);
    defer_max_s_   = declare_parameter<double>("reset_defer_max_s", 120.0);
    // ESCALATED re-arm: grant one fresh attempt after this long, else a camera
    // that had a transient bad spell is held out forever (0 = stay sticky).
    escalated_retry_s_ = declare_parameter<double>("escalated_retry_s", 600.0);
    status_period_s_ = declare_parameter<double>("status_period_s", 15.0);

    open_log();
    load_config();
    // No cameras (unprovisioned config, blank serials) must not kill the node: it
    // would take the watchdog out silently while bringup still reports success.
    // Idle loudly instead - a later provision + relaunch arms it.
    if (cams_.empty()) {
      log_line("NO CAMERAS parsed from " + config_path_ +
               " (blank serial_no?) - sentry idle, no watchdog until provisioned");
    }

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    for (auto &kv : cams_) {
      const std::string ns = kv.second.ns;
      for (const char *stream : {"infra1", "infra2"}) {
        std::string topic = "/" + ns + "/" + stream + "/camera_info";
        subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
            topic, sensor_qos,
            [this, name = kv.first, s = std::string(stream)]
            (sensor_msgs::msg::CameraInfo::ConstSharedPtr) {
              std::lock_guard<std::mutex> lk(mtx_);
              cams_[name].counts[s]++;
            }));
      }
    }

    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/visual_slam/tracking/odometry", sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          double t = rclcpp::Time(msg->header.stamp).seconds();
          if (vo_last_stamp_ > 0.0) {
            double gap = (t - vo_last_stamp_) * 1000.0;
            if (gap > vo_window_max_gap_ms_) vo_window_max_gap_ms_ = gap;
          }
          vo_last_stamp_ = t;
          vo_count_++;
        });

    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_pub_  = create_publisher<std_msgs::msg::String>("/vslam_sentry/status", latched);
    healthy_pub_ = create_publisher<std_msgs::msg::Bool>("/vslam_sentry/healthy", latched);

    for (auto &kv : cams_) {
      // Primary reset path: the DRIVER's hw_reset service (fork patch). The driver
      // owns the device handle; a second librealsense client cannot open a held
      // device (RS2_USB_STATUS_BUSY - proven live).
      hw_reset_clients_[kv.first] = create_client<std_srvs::srv::Trigger>(
          "/" + kv.second.ns + "/hw_reset");
    }
    for (auto &kv : cams_) {
      const std::string name = kv.first;
      reset_srvs_.push_back(create_service<std_srvs::srv::Trigger>(
          "/vslam_sentry/reset_" + name,
          [this, name](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            manual_reset(name, res);
          }));
    }
    status_srv_ = create_service<std_srvs::srv::Trigger>(
        "/vslam_sentry/status_now",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          res->success = true;
          res->message = build_status_json();
        });

    start_steady_ = steady_now();
    last_tick_steady_ = start_steady_;
    timer_ = create_wall_timer(
        std::chrono::duration<double>(tick_s_), [this] { tick(); });

    std::ostringstream ss;
    ss << "sentry up: cameras=";
    for (auto &kv : cams_) ss << kv.first << "(" << kv.second.serial << ") ";
    ss << "auto_reset=" << (auto_reset_ ? "on" : "off")
       << " settle=" << settle_s_ << "s";
    log_line(ss.str());
  }

 private:
  struct Cam {
    std::string ns;        // e.g. front_realsense
    std::string serial;
    std::map<std::string, int64_t> counts;      // stream -> msgs this window
    std::map<std::string, double>  hz;          // stream -> last window rate
    std::string port;                           // learned at reset time only
    double window_min_hz = 0.0;
    int    state = SETTLING;
    int    zero_ticks = 0;
    int    degraded_ticks = 0;
    bool   seen_streaming = false;
    int    attempts = 0;                        // auto attempts; manual never clears
    double last_reset_steady = 0.0;
    double recover_deadline = 0.0;
    double first_eligible_steady = 0.0;         // defer-ceiling anchor
    std::string last_reset_result = "none";
  };

  // ---- config / logging -------------------------------------------------------

  void load_config() {
    if (config_path_.empty()) return;
    YAML::Node root = YAML::LoadFile(config_path_);
    for (auto it = root.begin(); it != root.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      if (key.find("_realsense/") == std::string::npos) continue;
      YAML::Node p = it->second["ros__parameters"];
      if (!p) continue;
      Cam c;
      c.ns = p["camera_name"] ? p["camera_name"].as<std::string>()
                              : key.substr(0, key.find('/'));
      c.serial = p["serial_no"] ? p["serial_no"].as<std::string>() : "";
      if (c.serial.empty()) continue;
      // Pre-seed both streams so a stream that NEVER delivers still reads 0 Hz
      // (an empty map would hide it from the min() sweep).
      c.counts["infra1"] = 0;
      c.counts["infra2"] = 0;
      std::string short_name = c.ns.substr(0, c.ns.find('_'));
      cams_[short_name] = c;
    }
  }

  void open_log() {
    const char *ws = std::getenv("ISAAC_ROS_WS");
    std::string root = ws ? ws : "/workspaces/isaac_ros-dev";
    std::string dir = root + "/run_logs/sentry";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    log_path_ = dir + "/sentry.log";
    // Rotate one deep then truncate: bounded like the sibling vslam.log
    // (append-forever was a slow leak on a long-lived stack).
    std::error_code rec;
    std::filesystem::rename(log_path_, dir + "/sentry.prev.log", rec);
    log_.open(log_path_, std::ios::trunc);
    if (ec || !log_.is_open())
      RCLCPP_WARN(get_logger(), "sentry.log unavailable at %s (%s) - journal only",
                  log_path_.c_str(),
                  ec ? ec.message().c_str() : "open failed");
  }

  void log_line(const std::string &msg) {
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    if (log_.is_open()) {
      log_ << wall_stamp() << " " << msg << "\n";
      log_.flush();
    }
  }

  // ---- reset ------------------------------------------------------------------
  // The ONLY bus touch in the node. Scoped context: constructed, queried once,
  // destroyed. Returns: 1 = reset issued, 0 = serial not on bus (true GONE),
  // -1 = context/query error. Every call - regardless of outcome - arms the
  // bus-wide quarantine, so no two enumerations can happen closer than
  // quarantine_s apart (including two cameras failing in the same tick).
  int issue_reset(const std::string &serial, std::string &err) {
    try {
      rs2::context ctx;
      for (auto &&dev : ctx.query_devices()) {
        const char *ser = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (ser && serial == ser) {
          std::string port = dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)
                                 ? dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) : "";
          last_reset_port_ = port;
          dev.hardware_reset();
          return 1;
        }
      }
      err = "serial " + serial + " not on bus";
      return 0;
    } catch (const std::exception &e) {
      err = e.what();
      return -1;
    }
  }

  // Caller holds mtx_. Primary path: driver hw_reset service (driver owns the
  // device handle). Fallback: scoped direct rs2 reset, ONLY when the driver's
  // service is off the graph (driver dead -> device likely unclaimed).
  void begin_recovery(const std::string &name, bool manual) {
    Cam &c = cams_[name];
    // Bus/driver-wide spacing for EVERY attempt outcome: blocks any other
    // camera's begin_recovery (auto or manual) for quarantine_s, including
    // later cameras in this same tick's loop.
    quarantine_until_ = steady_now() + quarantine_s_;
    c.last_reset_steady = steady_now();
    const std::string tag = manual ? "manual" : "auto";

    auto client = hw_reset_clients_[name];
    if (client && client->service_is_ready()) {
      active_reset_ = name;
      c.state = RECOVERING;
      if (!manual) c.attempts++;
      c.recover_deadline = steady_now() + verify_s_;
      c.last_reset_result = "in-progress (driver)";
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      const uint64_t seq = ++reset_seq_;
      client->async_send_request(req,
          [this, name, seq](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
            std::lock_guard<std::mutex> lk(mtx_);
            if (seq != reset_seq_) return;  // stale response from a superseded attempt
            auto &cam = cams_[name];
            try {
              auto res = fut.get();
              cam.last_reset_result = res->success
                  ? "in-progress (driver confirmed)"
                  : "driver refused: " + res->message;
              log_line("RESET " + name + " driver response: " +
                       (res->success ? "hardware_reset issued" : res->message));
            } catch (const std::exception &e) {
              // Response lost = expected when the reset drops the device and the
              // driver tears the node down mid-reply; the stream verify decides.
              log_line("RESET " + name + " driver response lost (" +
                       std::string(e.what()) + ") - stream verify will decide");
            }
          });
      log_line("RESET " + name + " (" + tag + ") requested via driver hw_reset, "
               "verify window " + std::to_string((int)verify_s_) + "s");
      return;
    }

    // Driver service absent - driver node dead or never up for this camera.
    std::string err;
    int rc = issue_reset(c.serial, err);
    if (rc == 1) {
      active_reset_ = name;
      c.state = RECOVERING;
      if (!manual) c.attempts++;
      c.recover_deadline = steady_now() + verify_s_;
      c.port = last_reset_port_;
      c.last_reset_result = "in-progress (direct)";
      log_line("RESET " + name + " (" + tag + ") issued DIRECT (driver service "
               "absent): hardware_reset serial " + c.serial + " port " + c.port +
               ", verify window " + std::to_string((int)verify_s_) + "s");
      return;
    }
    // attempts is the total-recovery-attempt bound, charged for futile lookups
    // and context errors too - a permanently off-bus camera must stop costing
    // bus enumerations after max_attempts and sit ESCALATED for hardware.
    c.attempts++;
    if (rc == 0) {
      c.state = GONE;
      c.last_reset_result = "device off bus";
      log_line("RESET " + name + " (" + tag + ") NOT ISSUED: " + err +
               " - device off bus (harness/contact class), reset cannot help; "
               "hardware attention required");
    } else {
      c.last_reset_result = "issue-failed: " + err;
      log_line("RESET " + name + " (" + tag + ") FAILED to issue: " + err);
    }
  }

  void manual_reset(const std::string &name,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (steady_now() - start_steady_ < settle_s_) {
      res->success = false;
      res->message = "refused: sentry settling (" +
                     std::to_string((int)(settle_s_ - (steady_now() - start_steady_))) +
                     "s left) - bringup owns the cameras right now";
      return;
    }
    if (!active_reset_.empty()) {
      res->success = false;
      res->message = "reset already in progress on " + active_reset_ +
                     " - one camera at a time";
      return;
    }
    if (steady_now() < quarantine_until_) {
      res->success = false;
      res->message = "refused: post-reset quarantine (" +
                     std::to_string((int)(quarantine_until_ - steady_now())) +
                     "s left) - previous reset still settling on the bus";
      return;
    }
    begin_recovery(name, true);
    Cam &c = cams_[name];
    res->success = (c.state == RECOVERING);
    res->message = res->success ? ("reset issued on " + name + " (" + c.serial + ")")
                                : c.last_reset_result;
  }

  // ---- tick -------------------------------------------------------------------

  void tick() {
    std::lock_guard<std::mutex> lk(mtx_);

    // Real elapsed window, not the nominal period: a late executor tick followed
    // by a short catch-up tick would otherwise manufacture a fake low-Hz window
    // and auto-reset a healthy camera under CPU load.
    const double t = steady_now();
    double dt = t - last_tick_steady_;
    last_tick_steady_ = t;
    if (dt < 0.2) dt = 0.2;
    // Both sides: a stretched window fakes low Hz; the short catch-up tick after
    // it under-reports the same way.
    const bool window_sane = dt > tick_s_ * 0.5 && dt < tick_s_ * 1.5;

    const double elapsed = t - start_steady_;
    const bool settled = elapsed >= settle_s_;

    // window rates
    for (auto &kv : cams_) {
      Cam &c = kv.second;
      double worst = 1e9;
      for (auto &sc : c.counts) {
        c.hz[sc.first] = sc.second / dt;
        worst = std::min(worst, c.hz[sc.first]);
        sc.second = 0;
      }
      c.window_min_hz = (worst == 1e9) ? 0.0 : worst;
      if (c.window_min_hz > 0.0) c.seen_streaming = true;
    }
    const bool vo_silent_window = (vo_count_ == 0);
    vo_hz_ = vo_count_ / dt;
    double vo_gap = vo_window_max_gap_ms_;
    vo_count_ = 0;
    vo_window_max_gap_ms_ = 0.0;
    // Break the stamp chain across a silent window so the first message after an
    // outage does not synthesize a gap spanning the whole outage.
    if (vo_silent_window) vo_last_stamp_ = 0.0;

    // vslam classification. An anomalous window (late tick, e.g. this node's own
    // blocking bus enumeration or CPU saturation) starves this node's OWN
    // subscriptions - a zero-VO reading from such a window is a measurement
    // artifact, not evidence. Hold the previous state for that window.
    int prev_v = vslam_state_;
    if (!settled) vslam_state_ = V_SETTLING;
    else if (!window_sane) { /* hold previous classification */ }
    else if (vo_hz_ <= 0.01) vslam_state_ = V_DOWN;
    else if (vo_hz_ < vo_min_hz_ || vo_gap > vo_max_gap_ms_) vslam_state_ = V_STARVED;
    else vslam_state_ = V_OK;
    if (vslam_state_ != prev_v && settled) {
      std::ostringstream ss;
      ss << "vslam: " << vslam_name(prev_v) << " -> " << vslam_name(vslam_state_)
         << " (vo " << vo_hz_ << " Hz, max gap " << (int)vo_gap << " ms)";
      log_line(ss.str());
    }
    // Calm streak for reset deferral: consecutive sane V_OK windows.
    if (vslam_state_ == V_OK && window_sane) {
      vslam_ok_streak_++;
    } else {
      vslam_ok_streak_ = 0;
    }

    // per-camera classification + recovery bookkeeping
    for (auto &kv : cams_) {
      Cam &c = kv.second;
      int prev = c.state;

      if (c.state == RECOVERING) {
        // Streams flowing at rate ARE the proof of recovery - data cannot flow
        // from a device that is off the bus or a driver that has not reopened.
        // Positive evidence counts in any window; the FAILURE verdict is only
        // allowed on a sane window (a distorted post-reset window under-reads).
        // The dead-time guard rejects the window that STRADDLES the reset:
        // pre-reset frames in it average to a passing rate before the device
        // has even dropped (observed live: "verified" 1.6s after issue).
        if (steady_now() - c.last_reset_steady > reset_dead_time_s_) {
        if (c.window_min_hz >= min_hz_) {
          c.state = HEALTHY;
          c.attempts = 0;
          c.first_eligible_steady = 0.0;
          c.last_reset_result = "recovered";
          active_reset_.clear();
          quarantine_until_ = steady_now() + quarantine_s_;
          log_line("RESET " + kv.first + " VERIFIED: streams back at " +
                   std::to_string(c.window_min_hz) + " Hz");
        } else if ((window_sane && steady_now() > c.recover_deadline) ||
                   steady_now() > c.recover_deadline + verify_s_) {
          // Second clause: hard escape. Sustained window-insanity must not hold
          // RECOVERING (and the global reset token) forever on a dead camera.
          c.state = STREAM_DEAD;
          c.last_reset_result = "verify-timeout";
          active_reset_.clear();
          quarantine_until_ = steady_now() + quarantine_s_;
          log_line("RESET " + kv.first + " FAILED verify: min_hz=" +
                   std::to_string(c.window_min_hz) + " after " +
                   std::to_string((int)verify_s_) + "s");
        }
        }
        continue;
      }

      if (!settled && !c.seen_streaming) { c.state = SETTLING; continue; }

      if (c.window_min_hz >= min_hz_) {
        // Positive evidence clears every bad state incl. ESCALATED/GONE. The
        // defer anchor too: stale, it makes the next fault's ceiling instantly true.
        c.zero_ticks = 0;
        c.degraded_ticks = 0;
        c.state = HEALTHY;
        c.attempts = 0;
        c.first_eligible_steady = 0.0;
      } else if (!window_sane) {
        // Anomalous window (late tick / executor stall): the same stall that
        // stretched the window also starved this node's own subscription
        // callbacks, so a zero or low reading is not evidence of anything.
        // Accrue NOTHING this tick - a genuinely dead camera just waits one
        // extra window.
      } else if (c.state == ESCALATED || c.state == GONE) {
        // Sticky until real recovery - no reclassification churn. GONE keeps
        // its harness-class diagnostic instead of decaying into STREAM_DEAD.
      } else if (c.window_min_hz <= 0.01) {
        c.degraded_ticks = 0;
        c.zero_ticks++;
        if (c.zero_ticks >= dead_ticks_) c.state = STREAM_DEAD;
      } else {
        c.zero_ticks = 0;
        c.degraded_ticks++;
        if (c.degraded_ticks >= degraded_ticks_) c.state = DEGRADED;
      }

      if (c.state != prev) {
        std::ostringstream ss;
        ss << kv.first << ": " << state_name(prev) << " -> " << state_name(c.state)
           << " (min stream " << c.window_min_hz << " Hz)";
        log_line(ss.str());
      }

      // Auto recovery: one camera at a time (active_reset_ token + the bus-wide
      // quarantine armed by EVERY begin_recovery call, which also spaces two
      // same-tick candidates). GONE retries re-check the bus via the issue-time
      // lookup; attempts bound EVERY outcome, so a permanently off-bus camera
      // stops costing bus enumerations at the ceiling and sits ESCALATED.
      if (auto_reset_ && active_reset_.empty() && settled &&
          steady_now() >= quarantine_until_ &&
          (c.state == STREAM_DEAD || c.state == DEGRADED || c.state == GONE ||
           c.state == ESCALATED)) {
        if (c.attempts >= max_attempts_) {
          if (c.state != ESCALATED) {
            c.state = ESCALATED;
            log_line(kv.first + ": ESCALATED - " + std::to_string(max_attempts_) +
                     " recovery attempts exhausted (last: " + c.last_reset_result +
                     "); manual /vslam_sentry/reset_" + kv.first +
                     ", supervisor vslam cycle, or hardware attention required");
          } else if (escalated_retry_s_ > 0.0 &&
                     steady_now() - c.last_reset_steady > escalated_retry_s_) {
            c.attempts = max_attempts_ - 1;
            log_line(kv.first + ": ESCALATED re-arm after " +
                     std::to_string((int)escalated_retry_s_) + "s - one fresh attempt");
          }
        } else if (steady_now() - c.last_reset_steady > cooldown_s_) {
          // Calm-window deferral: fire on a calm VO streak, a second unhealthy
          // camera, or the bounded ceiling. Disabled at the launch site on this
          // single-camera rig (deferral cannot help when the one camera IS the VO).
          if (c.first_eligible_steady <= 0.0) {
            c.first_eligible_steady = steady_now();
          }
          // Only actionable states count: GONE/ESCALATED can't be helped by a
          // reset and would calm-bypass every sibling reset.
          int unhealthy = 0;
          for (const auto & uc : cams_) {
            if (uc.second.state == STREAM_DEAD || uc.second.state == DEGRADED) {unhealthy++;}
          }
          const bool calm = vslam_ok_streak_ >= calm_ticks_;
          const bool ceiling = defer_max_s_ > 0.0 &&
            steady_now() - c.first_eligible_steady > defer_max_s_;
          if (defer_max_s_ <= 0.0 || calm || unhealthy >= 2 || ceiling) {
            c.first_eligible_steady = 0.0;
            begin_recovery(kv.first, false);
          } else if (steady_now() - last_defer_log_ > 15.0) {
            last_defer_log_ = steady_now();
            log_line("RESET " + kv.first + " DEFERRED: VO not calm (" +
                     std::to_string(vslam_ok_streak_) + "/" +
                     std::to_string(calm_ticks_) + " calm windows) - waiting for a "
                     "quiet window (" +
                     std::to_string((int)(steady_now() - c.first_eligible_steady)) +
                     "s held, ceiling " + std::to_string((int)defer_max_s_) + "s)");
          }
        }
      }
    }

    publish_status(settled);
  }

  const char *vslam_name(int v) {
    switch (v) {
      case V_SETTLING: return "SETTLING";
      case V_OK: return "OK";
      case V_STARVED: return "STARVED";
      case V_DOWN: return "DOWN";
    }
    return "?";
  }

  static double fin(double v) { return std::isfinite(v) ? v : 0.0; }

  // Minimal JSON string escape: reset-failure text carries verbatim rs2::error
  // strings (quotes, backslashes) - exactly when a consumer most needs to parse
  // the status, so it must never break the JSON.
  static std::string jesc(const std::string &s) {
    std::string o;
    o.reserve(s.size());
    for (char ch : s) {
      if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
      else if (static_cast<unsigned char>(ch) < 0x20) { o += ' '; }
      else o += ch;
    }
    return o;
  }

  // Caller holds mtx_.
  std::string build_status_json() {
    std::ostringstream ss;
    ss << "{\"vslam\":{\"state\":\"" << vslam_name(vslam_state_)
       << "\",\"vo_hz\":" << fin(vo_hz_) << "},\"cameras\":{";
    bool first = true;
    for (auto &kv : cams_) {
      Cam &c = kv.second;
      if (!first) ss << ",";
      first = false;
      ss << "\"" << kv.first << "\":{\"state\":\"" << state_name(c.state)
         << "\",\"serial\":\"" << jesc(c.serial) << "\",\"port\":\"" << jesc(c.port) << "\"";
      for (auto &h : c.hz)
        ss << ",\"" << h.first << "_hz\":" << fin(h.second);
      ss << ",\"reset_attempts\":" << c.attempts
         << ",\"last_reset\":\"" << jesc(c.last_reset_result) << "\"}";
    }
    ss << "}}";
    return ss.str();
  }

  void publish_status(bool settled) {
    bool healthy = settled && vslam_state_ == V_OK;
    for (auto &kv : cams_)
      healthy = healthy && kv.second.state == HEALTHY;

    std::string j = build_status_json();
    const double t = steady_now();
    if (j != last_status_json_ || t - last_status_pub_ > status_period_s_) {
      std_msgs::msg::String m;
      m.data = j;
      status_pub_->publish(m);
      last_status_json_ = j;
      last_status_pub_ = t;
    }
    if (healthy != last_healthy_ || first_health_) {
      std_msgs::msg::Bool b;
      b.data = healthy;
      healthy_pub_->publish(b);
      last_healthy_ = healthy;
      first_health_ = false;
      log_line(std::string("aggregate healthy = ") + (healthy ? "TRUE" : "FALSE"));
    }
  }

  // ---- members ----------------------------------------------------------------

  std::string config_path_, log_path_;
  double settle_s_, tick_s_, min_hz_, vo_min_hz_, vo_max_gap_ms_;
  double cooldown_s_, verify_s_, reset_dead_time_s_, quarantine_s_, status_period_s_;
  double defer_max_s_;
  double escalated_retry_s_;
  int dead_ticks_, degraded_ticks_, max_attempts_, calm_ticks_;
  int vslam_ok_streak_ = 0;
  double last_defer_log_ = 0.0;
  bool auto_reset_;

  std::map<std::string, Cam> cams_;
  std::mutex mtx_;
  uint64_t reset_seq_ = 0;
  std::string active_reset_;
  std::string last_reset_port_;
  double quarantine_until_ = 0.0;

  std::ofstream log_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr healthy_pub_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> reset_srvs_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> hw_reset_clients_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  double start_steady_ = 0.0, last_tick_steady_ = 0.0;
  int vslam_state_ = V_SETTLING;
  int64_t vo_count_ = 0;
  double vo_hz_ = 0.0, vo_last_stamp_ = 0.0, vo_window_max_gap_ms_ = 0.0;
  std::string last_status_json_;
  double last_status_pub_ = 0.0;
  bool last_healthy_ = false, first_health_ = true;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VslamSentry>());
  } catch (const std::exception &e) {
    fprintf(stderr, "vslam_sentry fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
