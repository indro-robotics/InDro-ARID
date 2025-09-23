#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <filesystem> 
#include <yaml-cpp/yaml.h>

class CameraInfoPublisher : public rclcpp::Node
{
public:
    CameraInfoPublisher() : Node("camera_info_publisher")
    {
        this->declare_parameter<std::string>("calibration_file", "");
        this->declare_parameter<std::string>("camera_topic", "camera");
        this->declare_parameter<double>("update_rate", 1.0);
        
        std::string calibration_file = "file://" + this->get_parameter("calibration_file").as_string();
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        double update_rate = this->get_parameter("update_rate").as_double();

        // Print the full path of the calibration file
        std::filesystem::path full_path = std::filesystem::absolute(calibration_file);
        RCLCPP_INFO(this->get_logger(), "Full path of calibration file: %s", full_path.string().c_str());

        camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
        if (camera_info_manager_->loadCameraInfo(calibration_file)) {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded camera info from: %s", full_path.string().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera info from: %s", full_path.string().c_str());
        }

        // Create the full topic name
        std::string full_topic = camera_topic + "/camera_info";

        publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(full_topic, 2);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate),
            std::bind(&CameraInfoPublisher::publish_camera_info, this));
    }

    private:
      void publish_camera_info()
      {
          auto camera_info = camera_info_manager_->getCameraInfo();
          camera_info.header.stamp = this->now();
          publisher_->publish(camera_info);
          RCLCPP_INFO(this->get_logger(), "Published camera_info");
      }

      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
      rclcpp::TimerBase::SharedPtr timer_;
      std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  };

  int main(int argc, char** argv)
  {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<CameraInfoPublisher>();
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
  }
