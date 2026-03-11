#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>

/**
 * PipelineHeartbeatNode
 *
 * Composable node intended to live inside the same composable container as the
 * pipeline nodes (e.g. format_converter in cypher_argus / realsense_cv_pipe).
 * Subscribes to a configurable topic (typically image_rect) via intra-process
 * communication — zero-copy, no serialization overhead. Publishes a tiny
 * std_msgs/Empty on "heartbeat" (relative to node namespace) for every received
 * message, giving external monitors a lightweight proxy for pipeline health.
 *
 * Parameters:
 *   watch_topic  (string, default "image_rect")  — topic to subscribe to
 *
 * Published topics:
 *   ~heartbeat   std_msgs/Empty — one message per received watch_topic message
 */
class PipelineHeartbeatNode : public rclcpp::Node
{
public:
  explicit PipelineHeartbeatNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pipeline_heartbeat", options)
  {
    this->declare_parameter<std::string>("watch_topic", "image_rect");
    std::string watch_topic = this->get_parameter("watch_topic").as_string();

    heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>("heartbeat", 10);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      watch_topic,
      rclcpp::SensorDataQoS(),  // BEST_EFFORT+VOLATILE — connects to any publisher QoS
      [this](sensor_msgs::msg::Image::ConstSharedPtr) {
        heartbeat_pub_->publish(std_msgs::msg::Empty{});
      }
    );

    RCLCPP_INFO(this->get_logger(), "Watching '%s' → publishing heartbeat", watch_topic.c_str());
  }

private:
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PipelineHeartbeatNode)
