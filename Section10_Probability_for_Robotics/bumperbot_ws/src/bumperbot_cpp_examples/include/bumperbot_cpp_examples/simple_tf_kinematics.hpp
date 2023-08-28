#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "bumperbot_msgs/srv/get_transform.hpp"

#include <memory>


class SimpleTfKinematics : public rclcpp::Node
{
public:
    SimpleTfKinematics(const std::string& name);

private: 
    void timerCallback();

    bool getTransformCallback(const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> req,
                              const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> res);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<bumperbot_msgs::srv::GetTransform>::SharedPtr get_transform_srv_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;
    double last_x_;
    double x_increment_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2::Quaternion last_orientation_;
    tf2::Quaternion orientation_increment_;
    int rotations_counter_;
};

#endif // SIMPLE_TF_KINEMATICS_HPP