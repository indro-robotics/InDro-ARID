#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <mutex>

using std::placeholders::_1;

class SynchronNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr synced_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr synced_info_pub_;
    
    sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
    
    std::mutex image_mutex_;
    std::mutex info_mutex_;
    
    std::string frame_id_;

public:
    SynchronNode() : Node("synchron_node") {
        // ROS2 parameter declaration
        this->declare_parameter<std::string>("frame_id", "camera_frame");
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // ROS2 QoS setup (important for image transport)
        auto qos = rclcpp::SensorDataQoS().reliable();
        
        // ROS2 subscribers
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos,
            std::bind(&SynchronNode::imageCallback, this, _1));
            
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", qos,
            std::bind(&SynchronNode::cameraInfoCallback, this, _1));
            
        // ROS2 publishers
        synced_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "synced_image", qos);
        synced_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "synced_camera_info", qos);
            
        RCLCPP_INFO(this->get_logger(), 
            "Synchronization node initialized with frame_id: %s", 
            frame_id_.c_str());
    }
    
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = msg;
        }
        publishSynchronizedMessages();
    }
    
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(info_mutex_);
            latest_camera_info_ = msg;
        }
    }
    
    void publishSynchronizedMessages() {
        sensor_msgs::msg::Image::ConstSharedPtr image;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
        
        {
            std::lock_guard<std::mutex> lock_image(image_mutex_);
            std::lock_guard<std::mutex> lock_info(info_mutex_);
            
            if (!latest_image_ || !latest_camera_info_) {
                return;
            }
            
            image = latest_image_;
            info = latest_camera_info_;
        }
        
        // Create modified messages
        auto synced_image = *image;
        auto synced_info = *info;
        
        // Apply frame_id parameter
        synced_image.header.frame_id = frame_id_;
        synced_info.header.frame_id = frame_id_;
        
        // Synchronize timestamps
        synced_info.header.stamp = synced_image.header.stamp;
        
        synced_image_pub_->publish(synced_image);
        synced_info_pub_->publish(synced_info);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynchronNode>());
    rclcpp::shutdown();
    return 0;
}
