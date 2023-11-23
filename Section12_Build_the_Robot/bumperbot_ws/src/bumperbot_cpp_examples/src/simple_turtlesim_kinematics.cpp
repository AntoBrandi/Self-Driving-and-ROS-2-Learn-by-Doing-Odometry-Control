#include "bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp"

using std::placeholders::_1;


SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string& name)
    : Node(name)
{
    turtle1_pose_sub_ = create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback, this, _1));

    turtle2_pose_sub_ = create_subscription<turtlesim::msg::Pose>(
        "/turtle2/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, _1));
}


void SimpleTurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose& pose)
{
    last_turtle1_pose_ = pose;
}


void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose& pose)
{
    last_turtle2_pose_ = pose;
    float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;
    float theta_rad = last_turtle2_pose_.theta - last_turtle1_pose_.theta;
    float theta_deg = 180 * theta_rad / 3.14;
    RCLCPP_INFO_STREAM(get_logger(), "\nTranslation Vector turtle1 -> turtle2 \n" <<
                    "Tx: " << Tx << "\n" <<
                    "Ty: " << Ty << "\n" <<
                    "Rotation Matrix turtle1 -> turtle2 \n" << 
                    "theta (rad): " << theta_rad << "\n" <<
                    "theta (deg): " << theta_deg << "\n" <<
                    "|R11   R12|:  |" << std::cos(theta_rad) << "\t" << -std::sin(theta_rad) << "|\n" <<
                    "|R21   R22|   |" << std::sin(theta_rad) << "\t\t" << std::cos(theta_rad) << "|\n");
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}