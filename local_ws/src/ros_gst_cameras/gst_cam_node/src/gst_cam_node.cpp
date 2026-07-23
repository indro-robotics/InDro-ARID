#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <thread>
#include <atomic>
#include <cstring>
#include <string>

/**
 * Wraps the GStreamer pipeline given in gst_pipeline; publishes /<camera_topic>/image_raw
 * (+ compressed when compress:=true) and camera_info synced to each frame.
 */
class GstCamNode : public rclcpp::Node
{
public:
  explicit GstCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gst_cam_node", options), running_(true), has_calibration_(false)
  {
    this->declare_parameter<std::string>("gst_pipeline", "");
    this->declare_parameter<std::string>("camera_topic", "cam_down");
    this->declare_parameter<std::string>("frame_id", "camera_frame");
    this->declare_parameter<std::string>("camera_info_path", "");
    this->declare_parameter<std::string>("encoding", "");
    this->declare_parameter<bool>("compress", true);
    this->declare_parameter<bool>("reliable", false);

    std::string camera_topic     = this->get_parameter("camera_topic").as_string();
    std::string camera_info_path = this->get_parameter("camera_info_path").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    encoding_ = this->get_parameter("encoding").as_string();
    bool compress = this->get_parameter("compress").as_bool();
    bool reliable = this->get_parameter("reliable").as_bool();

    // Default sensor_data QoS; reliable:=true for streams that cannot drop frames.
    rclcpp::QoS image_qos = rclcpp::QoS(rclcpp::KeepLast(5)).durability_volatile();
    if (reliable) {
      image_qos.reliable();
    } else {
      image_qos.best_effort();
    }
    rmw_qos_profile_t image_qos_profile = image_qos.get_rmw_qos_profile();

    if (compress) {
      it_pub_ = image_transport::create_publisher(this, "/" + camera_topic + "/image_raw",
                                                  image_qos_profile);
    } else {
      image_pub_  = this->create_publisher<sensor_msgs::msg::Image>(
                    "/" + camera_topic + "/image_raw", image_qos);
    }

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                        "/" + camera_topic + "/camera_info", image_qos);

    RCLCPP_INFO(this->get_logger(), "QoS: %s", reliable ? "RELIABLE" : "BEST_EFFORT (sensor_data)");

    if (!camera_info_path.empty()) {
      camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
                               this, camera_topic);
      if (camera_info_manager_->validateURL(camera_info_path) &&
          camera_info_manager_->loadCameraInfo(camera_info_path))
      {
        has_calibration_ = true;
        RCLCPP_INFO(this->get_logger(), "Loaded calibration: %s", camera_info_path.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid calibration path: %s; publishing default camera_info",
                    camera_info_path.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No calibration specified; publishing default camera_info");
    }

    gst_thread_ = std::thread(&GstCamNode::start_gst_pipeline, this);
  }

  ~GstCamNode()
  {
    running_ = false;
    if (gst_thread_.joinable())
      gst_thread_.join();
  }

private:
  void start_gst_pipeline()
  {
    std::string pipeline = this->get_parameter("gst_pipeline").as_string();

    if (pipeline.empty()) {
      RCLCPP_ERROR(this->get_logger(), "gst_pipeline parameter is empty; nothing to open.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Opening pipeline:\n%s", pipeline.c_str());
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Pipeline open.");

    cv::Mat frame;
    while (rclcpp::ok() && running_) {
      if (!cap.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Frame read failed.");
        continue;
      }

      auto now = this->now();

      // Resolved once on the first frame: override if set, else auto-detect.
      if (!encoding_resolved_) {
        active_encoding_ = encoding_.empty() ? detect_encoding(frame) : encoding_;
        if (active_encoding_.empty()) {
          RCLCPP_WARN(this->get_logger(),
            "Could not map cv::Mat type %d to a ROS encoding; publishing with empty encoding. "
            "Set `encoding:` in the pipeline config to override.", frame.type());
        } else {
          RCLCPP_INFO(this->get_logger(), "Image encoding: %s (%s)",
                      active_encoding_.c_str(),
                      encoding_.empty() ? "auto-detected" : "override");
        }
        encoding_resolved_ = true;
      }

      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp    = now;
      msg->header.frame_id = frame_id_;
      msg->height          = static_cast<uint32_t>(frame.rows);
      msg->width           = static_cast<uint32_t>(frame.cols);
      msg->encoding        = active_encoding_;
      msg->is_bigendian    = false;
      msg->step            = static_cast<uint32_t>(frame.step);
      msg->data.resize(msg->step * msg->height);
      std::memcpy(msg->data.data(), frame.data, msg->data.size());
      if (image_pub_) {
        image_pub_->publish(std::move(msg));
      } else {
        it_pub_.publish(*msg);
      }

      // camera_info stamp must match the image stamp: subscribers pair them.
      sensor_msgs::msg::CameraInfo cam_info;
      if (has_calibration_) {
        cam_info = camera_info_manager_->getCameraInfo();
      } else {
        if (!default_info_ready_) {
          default_info_ = make_default_camera_info(frame.cols, frame.rows);
          default_info_ready_ = true;
        }
        cam_info = default_info_;
      }
      cam_info.header.stamp    = now;
      cam_info.header.frame_id = frame_id_;
      camera_info_pub_->publish(cam_info);
    }

    cap.release();
  }

  // Placeholder: zero distortion, fx = fy = width, principal point at image centre.
  static sensor_msgs::msg::CameraInfo make_default_camera_info(int width, int height)
  {
    sensor_msgs::msg::CameraInfo info;
    info.width  = static_cast<uint32_t>(width);
    info.height = static_cast<uint32_t>(height);
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    const double fx = static_cast<double>(width);
    const double fy = static_cast<double>(width);
    const double cx = width  / 2.0;
    const double cy = height / 2.0;

    info.k = {fx,  0.0, cx,
              0.0, fy,  cy,
              0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0};
    info.p = {fx,  0.0, cx,  0.0,
              0.0, fy,  cy,  0.0,
              0.0, 0.0, 1.0, 0.0};
    return info;
  }

  // "" when the cv::Mat type has no clean ROS mapping: warn, never silently mislabel.
  static std::string detect_encoding(const cv::Mat & frame)
  {
    switch (frame.type()) {
      case CV_8UC1:  return sensor_msgs::image_encodings::MONO8;
      case CV_8UC3:  return sensor_msgs::image_encodings::BGR8;
      case CV_8UC4:  return sensor_msgs::image_encodings::BGRA8;
      case CV_16UC1: return sensor_msgs::image_encodings::MONO16;
      case CV_16UC3: return sensor_msgs::image_encodings::BGR16;
      case CV_16UC4: return sensor_msgs::image_encodings::BGRA16;
      default:       return "";
    }
  }

  std::thread gst_thread_;
  std::atomic<bool> running_;
  bool has_calibration_;
  bool encoding_resolved_{false};
  bool default_info_ready_{false};
  std::string frame_id_;
  std::string encoding_;         // override (empty = auto-detect)
  std::string active_encoding_;  // resolved at first frame
  sensor_msgs::msg::CameraInfo default_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;  // compress=false
  image_transport::Publisher it_pub_;  // compress=true: raw + compressed (lazy encoder)
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GstCamNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GstCamNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
