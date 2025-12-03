#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <thread>
#include <atomic>
#include <string>

class GstCamNode : public rclcpp::Node
{
public:
  explicit GstCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gst_cam_node", options), running_(true)
  {
    // Declare parameters
    this->declare_parameter<int>("vid_src", 0);
    this->declare_parameter<int>("framerate", 10);
    this->declare_parameter<int>("width", 1640);
    this->declare_parameter<int>("height", 1232);
    this->declare_parameter<std::string>("camera_topic", "cam_down");
    this->declare_parameter<std::string>("frame_id", "camera_frame");
    this->declare_parameter<std::string>("camera_info_path", "");

    // QoS
    const rclcpp::QoS image_qos{rclcpp::QoS(3).reliable().durability_volatile()};

    // Get launch-time parameters
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string camera_info_path = this->get_parameter("camera_info_path").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Set up publishers
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                  "/" + camera_topic + "/image_raw", image_qos);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
                    "/" + camera_topic + "/camera_info", image_qos);

    // Initialize camera info manager
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_topic);
    if (!camera_info_path.empty()) {
      if (!camera_info_manager_->loadCameraInfo(camera_info_path)) {
        RCLCPP_WARN(this->get_logger(), "Failed to load camera_info from: %s", camera_info_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Loaded camera_info from: %s", camera_info_path.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "No camera_info_path provided. CameraInfo will use defaults.");
    }

    // Start GStreamer thread
    gst_thread_ = std::thread(&GstCamNode::start_gst_pipeline, this);

    RCLCPP_INFO(this->get_logger(), "gst pipeline opened");
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
    int vid_src = this->get_parameter("vid_src").as_int();
    int framerate = this->get_parameter("framerate").as_int();
    int width = this->get_parameter("width").as_int();
    int height = this->get_parameter("height").as_int();

    std::string gstreamer_pipeline =
      "nvarguscamerasrc sensor-id=" + std::to_string(vid_src) +
      " wbmode=1 aelock=false ee-mode=2 tnr-mode=2 ! "
      "video/x-raw(memory:NVMM),width=" + std::to_string(width) +
      ",height=" + std::to_string(height) +
      ",framerate=" + std::to_string(framerate) +
      "/1,format=NV12 ! "
      "nvvidconv flip-method=0 interpolation-method=1 ! "
      "video/x-raw,format=GRAY8 ! appsink";

    cv::VideoCapture cap(gstreamer_pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera pipeline:\n%s", gstreamer_pipeline.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Camera pipeline opened:\n%s", gstreamer_pipeline.c_str());

    cv::Mat frame;
    rclcpp::WallRate rate(framerate);

    while (rclcpp::ok() && running_) {
      if (!cap.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Camera frame read failed.");
        continue;
      }
      // Create synchronized header: same for both msgs
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = frame_id_;

      // Publish image_raw
      auto msg = cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
      image_pub_->publish(*msg);

      // Publish camera_info with matching header
      auto cam_info_msg = camera_info_manager_->getCameraInfo();
      cam_info_msg.header.stamp = header.stamp;
      cam_info_msg.header.frame_id = header.frame_id;
      camera_info_pub_->publish(cam_info_msg);

      rate.sleep();
    }
  }

  // Member variables
  std::thread gst_thread_;
  std::atomic<bool> running_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::string frame_id_;
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
