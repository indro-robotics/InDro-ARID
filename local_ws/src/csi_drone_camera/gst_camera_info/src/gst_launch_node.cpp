#include "rclcpp_components/register_node_macro.hpp"
#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <cstdlib>

class GstLaunchNode : public rclcpp::Node
{
public:
  GstLaunchNode(const rclcpp::NodeOptions & options)
  : Node("gst_launch_node", options)
  {
    std::string pre_commands = this->declare_parameter<std::string>("pre_commands", "");
    std::string pipeline_str = this->declare_parameter<std::string>("gst_pipeline", "");
    
    // Execute pre-commands
    if (!pre_commands.empty()) {
      int result = system(pre_commands.c_str());
      if (result != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute pre-commands: %s", pre_commands.c_str());
        return;
      }
    }

    // Initialize GStreamer
    gst_init(nullptr, nullptr);
    
    // Create and start the pipeline
    GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), nullptr);
    if (!pipeline) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline");
      return;
    }

    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
      gst_object_unref(pipeline);
      return;
    }
    
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(GstLaunchNode)
