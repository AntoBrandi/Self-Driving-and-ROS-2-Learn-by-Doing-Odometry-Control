#include "bumperbot_cpp_examples/simple_tf_kinematics.hpp"

#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;


SimpleTfKinematics::SimpleTfKinematics(const std::string& name)
    : Node(name),
      last_x_(0.0),
      x_increment_(0.05),
      rotations_counter_(0)
{
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  static_transform_stamped_.header.stamp = get_clock()->now();
  static_transform_stamped_.header.frame_id = "bumperbot_base";
  static_transform_stamped_.child_frame_id = "bumperbot_top";
  static_transform_stamped_.transform.translation.x = 0;
  static_transform_stamped_.transform.translation.y = 0;
  static_transform_stamped_.transform.translation.z = 0.3;
  static_transform_stamped_.transform.rotation.x = 0;
  static_transform_stamped_.transform.rotation.y = 0;
  static_transform_stamped_.transform.rotation.z = 0;
  static_transform_stamped_.transform.rotation.w = 1;
  static_tf_broadcaster_->sendTransform(static_transform_stamped_);
  RCLCPP_INFO_STREAM(get_logger(), "Publishing static transform between " <<
                   static_transform_stamped_.header.frame_id << " and " << 
                   static_transform_stamped_.child_frame_id);

  // Timer
  timer_ = create_wall_timer(0.1s, std::bind(&SimpleTfKinematics::timerCallback, this));

  // Service Server
  get_transform_srv_ = create_service<bumperbot_msgs::srv::GetTransform>("get_transform", std::bind(&SimpleTfKinematics::getTransformCallback, this, _1, _2));

  // Quaternion
  last_orientation_.setRPY(0, 0, 0);
  orientation_increment_.setRPY(0, 0, 0.05); // in radians
}


void SimpleTfKinematics::timerCallback()
{  
  dynamic_transform_stamped_.header.stamp = get_clock()->now();
  dynamic_transform_stamped_.header.frame_id = "odom";
  dynamic_transform_stamped_.child_frame_id = "bumperbot_base";
  dynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_;
  dynamic_transform_stamped_.transform.translation.y = 0;
  dynamic_transform_stamped_.transform.translation.z = 0;
  // dynamic_transform_stamped_.transform.rotation.x = 0;
  // dynamic_transform_stamped_.transform.rotation.y = 0;
  // dynamic_transform_stamped_.transform.rotation.z = 0;
  // dynamic_transform_stamped_.transform.rotation.w = 1;

  // Euler to Quaternion
  tf2::Quaternion q;
  q = last_orientation_ * orientation_increment_;
  q.normalize(); // sum == 1
  dynamic_transform_stamped_.transform.rotation.x = q.x();
  dynamic_transform_stamped_.transform.rotation.y = q.y();
  dynamic_transform_stamped_.transform.rotation.z = q.z();
  dynamic_transform_stamped_.transform.rotation.w = q.w();

  dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);
  last_x_ = dynamic_transform_stamped_.transform.translation.x;
  last_orientation_ = q;
  rotations_counter_++;
  if(rotations_counter_ >= 100)
  {
    orientation_increment_ = orientation_increment_.inverse();
    rotations_counter_ = 0;
  }
}


bool SimpleTfKinematics::getTransformCallback(const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> req,
                                              const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> res)
{
  RCLCPP_INFO_STREAM(get_logger(), "Requested Transform between " << req->frame_id << " and " << req->child_frame_id);
  geometry_msgs::msg::TransformStamped requested_transform;
  try{
    requested_transform = tf_buffer_->lookupTransform(req->frame_id, 
                                                     req->child_frame_id, 
                                                     tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "An error occurred while transforming " 
                     << req->frame_id << " and " << req->child_frame_id
                     << ": " << ex.what());
    res->success = false;
    return true;
  }

  res->transform = requested_transform;
  res->success = true;
  return true;
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}