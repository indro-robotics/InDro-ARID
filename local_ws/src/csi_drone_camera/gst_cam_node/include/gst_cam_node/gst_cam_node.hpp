#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <string>

class GstCamNode : public rclcpp::Node
{
public:
  explicit GstCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GstCamNode();

private:
  void start_gst_pipeline();

  std::thread gst_thread_;
  std::atomic<bool> running_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};
