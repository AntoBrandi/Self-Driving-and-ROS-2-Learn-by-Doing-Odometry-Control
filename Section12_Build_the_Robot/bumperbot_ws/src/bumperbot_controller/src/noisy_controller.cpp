#include "bumperbot_controller/noisy_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <random>


using std::placeholders::_1;


NoisyController::NoisyController(const std::string& name)
                                  : Node(name)
                                  , left_wheel_prev_pos_(0.0)
                                  , right_wheel_prev_pos_(0.0)
                                  , x_(0.0)
                                  , y_(0.0)
                                  , theta_(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&NoisyController::jointCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom_noisy", 10);

    // Fill the Odometry message with invariant parameters
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_ekf";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_noisy";

    prev_time_ = get_clock()->now();
}


void NoisyController::jointCallback(const sensor_msgs::msg::JointState &state)
{
    // Implements the inverse differential kinematic model
    // Given the position of the wheels, calculates their velocities
    // then calculates the velocity of the robot wrt the robot frame
    // and then converts it in the global frame and publishes the TF

    // Add noise to wheel readings
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
    std::normal_distribution<double> right_encoder_noise(0.0, 0.005);

    double wheel_encoder_left = state.position.at(1) + left_encoder_noise(noise_generator);
    double wheel_encoder_right = state.position.at(0) + right_encoder_noise(noise_generator);

    double dp_left = wheel_encoder_left - left_wheel_prev_pos_;
    double dp_right = wheel_encoder_right - right_wheel_prev_pos_;
    rclcpp::Time msg_time = state.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    // Actualize the prev pose for the next itheration
    left_wheel_prev_pos_ = state.position.at(1);
    right_wheel_prev_pos_ = state.position.at(0);
    prev_time_ = state.header.stamp;

    // Calculate the rotational speed of each wheel
    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();

    // Calculate the linear and angular velocity
    double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2;
    double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

    // Calculate the position increment
    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;
    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    // Compose and publish the odom message
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;
    odom_pub_->publish(odom_msg_);

    // TF
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();
    transform_broadcaster_->sendTransform(transform_stamped_);
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NoisyController>("noisy_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}