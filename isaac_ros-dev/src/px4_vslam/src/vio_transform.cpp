// PX4 external-vision bridge. Converts px4_vslam_reactor's gated odometry into
// px4_msgs/VehicleOdometry on /fmu/in/vehicle_visual_odometry, which uXRCE-DDS carries to EKF2.
// Launched by vslam.launch.py.
#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp"

class VioTransform : public rclcpp::Node
{
public:
explicit VioTransform() : Node("vio_transform")
{
	// BEST_EFFORT, matching px4_vslam_reactor's filt_slam_odometry publisher: a RELIABLE
	// subscription does not connect and EKF2 receives no vision at all. Depth 30 because VO
	// arrives in bursts that a shallower queue drops without error.
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 30), qos_profile);

	_vio_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

	_vslam_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/visual_slam/filt_slam_odometry", qos,
						std::bind(&VioTransform::odometryCallback, this, std::placeholders::_1));

	_vslam_status_sub = this->create_subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>("/visual_slam/status", qos,
						std::bind(&VioTransform::statusCallback, this, std::placeholders::_1));

	// The reactor's epoch becomes VehicleOdometry.reset_counter, so EKF2 re-anchors on a re-seat
	// instead of gating the jump as an outlier. Transient-local because the reactor publishes the
	// epoch once at startup: a volatile subscription misses it and stamps 0 until the next seat.
	_reset_epoch_sub = this->create_subscription<std_msgs::msg::UInt8>("/reactor/vio_reset_epoch",
						rclcpp::QoS(1).reliable().transient_local(),
						std::bind(&VioTransform::resetEpochCallback, this, std::placeholders::_1));
}

private:
	void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
	void statusCallback(const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::UniquePtr msg);
	void resetEpochCallback(const std_msgs::msg::UInt8::UniquePtr msg);
	void sensorCombinedCallback(const px4_msgs::msg::SensorCombined::UniquePtr msg);

	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _vio_pub;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _vslam_odom_sub;
	rclcpp::Subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::SharedPtr _vslam_status_sub;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _reset_epoch_sub;
	uint8_t _vslam_state = 0;
	uint8_t _reset_epoch = 0;
};

void VioTransform::statusCallback(const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::UniquePtr msg)
{
	if (msg->vo_state != _vslam_state) {
		RCLCPP_INFO(get_logger(), "[VioTransform] state change: %u", msg->vo_state);
	}

	_vslam_state = msg->vo_state;
}

void VioTransform::resetEpochCallback(const std_msgs::msg::UInt8::UniquePtr msg)
{
	if (msg->data != _reset_epoch) {
		RCLCPP_INFO(get_logger(), "[VioTransform] EV reset epoch: %u", msg->data);
	}

	_reset_epoch = msg->data;
}

void VioTransform::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg)
{
	tf2::Vector3 position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	tf2::Quaternion quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Vector3 velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
	tf2::Vector3 angular_velocity(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
	tf2::Vector3 position_variance(msg->pose.covariance[0], msg->pose.covariance[7], msg->pose.covariance[14]);
	tf2::Vector3 orientation_variance(msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35]);
	tf2::Vector3 velocity_variance(msg->twist.covariance[0], msg->twist.covariance[7], msg->twist.covariance[14]);

	// isaac_ros_visual_slam publishes Odometry in FLU; VehicleOdometry is FRD.
	tf2::Quaternion rotation;
	rotation.setRPY(M_PI, 0.0, 0.0);

	position = tf2::quatRotate(rotation, position);
	quaternion = rotation * quaternion * rotation.inverse();
	velocity = tf2::quatRotate(rotation, velocity);
	angular_velocity = tf2::quatRotate(rotation, angular_velocity);
	// Covariance diagonals, not vectors: the roll-pi rotation negates their y and z terms, so
	// without the magnitude EKF2 receives negative variances.
	position_variance = tf2::quatRotate(rotation, position_variance).absolute();
	orientation_variance = tf2::quatRotate(rotation, orientation_variance).absolute();
	velocity_variance = tf2::quatRotate(rotation, velocity_variance).absolute();

	px4_msgs::msg::VehicleOdometry vio;

	// PX4 timestamps are microseconds; the int32 seconds field overflows at wall-clock epoch
	// values unless it is widened before the multiply.
	vio.timestamp = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000ULL + msg->header.stamp.nanosec / 1000;
	vio.timestamp_sample = vio.timestamp;

	vio.pose_frame = vio.POSE_FRAME_FRD;

	vio.q[0] = quaternion.getW();
	vio.q[1] = quaternion.getX();
	vio.q[2] = quaternion.getY();
	vio.q[3] = quaternion.getZ();

	vio.position[0] = position.getX();
	vio.position[1] = position.getY();
	vio.position[2] = position.getZ();

	vio.velocity_frame = vio.VELOCITY_FRAME_BODY_FRD;
	vio.velocity[0] = velocity.getX();
	vio.velocity[1] = velocity.getY();
	vio.velocity[2] = velocity.getZ();

	vio.angular_velocity[0] = angular_velocity.getX();
	vio.angular_velocity[1] = angular_velocity.getY();
	vio.angular_velocity[2] = angular_velocity.getZ();

	vio.position_variance[0] = position_variance.getX();
	vio.position_variance[1] = position_variance.getY();
	vio.position_variance[2] = position_variance.getZ();

	vio.orientation_variance[0] = orientation_variance.getX();
	vio.orientation_variance[1] = orientation_variance.getY();
	vio.orientation_variance[2] = orientation_variance.getZ();

	vio.velocity_variance[0] = velocity_variance.getX();
	vio.velocity_variance[1] = velocity_variance.getY();
	vio.velocity_variance[2] = velocity_variance.getZ();

	vio.reset_counter = _reset_epoch;
	vio.quality = _vslam_state;

	_vio_pub->publish(vio);
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VioTransform>());
	rclcpp::shutdown();
	return 0;
}
