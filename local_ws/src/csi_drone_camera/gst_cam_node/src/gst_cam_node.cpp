#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <thread>
#include <atomic>
#include <cstring>
#include <string>

/**
 * GstCamNode: A ROS 2 Node that wraps an arbitrary GStreamer pipeline.
 * The full pipeline string is passed via the gst_pipeline parameter.
 * Publishes image_raw (and image_raw/compressed when compress:=true) under /<camera_topic>/.
 * Publishes camera_info synced to each frame only if a calibration file is provided.
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
    this->declare_parameter<std::string>("encoding", "bgr8");
    this->declare_parameter<bool>("compress", true);

    std::string camera_topic     = this->get_parameter("camera_topic").as_string();
    std::string camera_info_path = this->get_parameter("camera_info_path").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    encoding_ = this->get_parameter("encoding").as_string();
    bool compress = this->get_parameter("compress").as_bool();

    const rclcpp::QoS image_qos{rclcpp::QoS(3).reliable().durability_volatile()};

    if (compress) {
      it_pub_ = image_transport::create_publisher(this, "/" + camera_topic + "/image_raw");
    } else {
      image_pub_  = this->create_publisher<sensor_msgs::msg::Image>(
                    "/" + camera_topic + "/image_raw", image_qos);
    }

    // Only set up camera_info if a calibration file was provided
    if (!camera_info_path.empty()) {
      camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
                               this, camera_topic);
      if (camera_info_manager_->validateURL(camera_info_path) &&
          camera_info_manager_->loadCameraInfo(camera_info_path))
      {
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                            "/" + camera_topic + "/camera_info", image_qos);
        has_calibration_ = true;
        RCLCPP_INFO(this->get_logger(), "Loaded calibration: %s", camera_info_path.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid calibration path: %s — camera_info disabled",
                    camera_info_path.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No calibration specified — publishing image_raw only");
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
      RCLCPP_ERROR(this->get_logger(), "gst_pipeline parameter is empty — nothing to open.");
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

      // Fill image message directly from cv::Mat — one copy into msg->data,
      // then move ownership to the publisher (no further copy).
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp    = now;
      msg->header.frame_id = frame_id_;
      msg->height          = static_cast<uint32_t>(frame.rows);
      msg->width           = static_cast<uint32_t>(frame.cols);
      msg->encoding        = encoding_;
      msg->is_bigendian    = false;
      msg->step            = static_cast<uint32_t>(frame.step);
      msg->data.resize(msg->step * msg->height);
      std::memcpy(msg->data.data(), frame.data, msg->data.size());
      if (image_pub_) {
        image_pub_->publish(std::move(msg));
      } else {
        it_pub_.publish(*msg);
      }

      // Publish camera_info with the same timestamp — only if calibration was loaded
      if (has_calibration_) {
        auto cam_info = camera_info_manager_->getCameraInfo();
        cam_info.header.stamp    = now;
        cam_info.header.frame_id = frame_id_;
        camera_info_pub_->publish(cam_info);
      }
    }

    cap.release();
  }

  std::thread gst_thread_;
  std::atomic<bool> running_;
  bool has_calibration_;
  std::string frame_id_;
  std::string encoding_;
  // compress=false: raw publisher only
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  // compress=true: image_transport publishes both raw + compressed (lazy — no cost when unsubscribed)
  image_transport::Publisher it_pub_;
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
