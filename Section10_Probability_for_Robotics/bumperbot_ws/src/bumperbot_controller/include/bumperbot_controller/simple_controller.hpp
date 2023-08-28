#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


class SimpleController : public rclcpp::Node
{
public:
    SimpleController(const std::string& name);

private:
    void velCallback(const geometry_msgs::msg::TwistStamped &msg);

    void jointCallback(const sensor_msgs::msg::JointState &msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Odometry
    double wheel_radius_;
    double wheel_separation_;
    Eigen::Matrix2d speed_conversion_;
    double right_wheel_prev_pos_;
    double left_wheel_prev_pos_;
    rclcpp::Time prev_time_;
    nav_msgs::msg::Odometry odom_msg_;
    double x_;
    double y_;
    double theta_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif // SIMPLE_CONTROLLER_HPP