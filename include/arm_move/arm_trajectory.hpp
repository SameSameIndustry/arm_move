#ifndef MY_TRAJECTORY_SENDER__ARM_TRAJECTORY_HPP_
#define MY_TRAJECTORY_SENDER__ARM_TRAJECTORY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/empty.hpp"

namespace arm_move
{

class ArmTrajectory : public rclcpp::Node
{
public:
  ArmTrajectory(const rclcpp::NodeOptions & options);

private:
  void timer_callback();

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr turn_table_position_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_pose_subscriber_; // TODO:本来ゴーをもらう型にする
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_motion_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr init_motion_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  std::vector<std::string> joint_names_;
  double l1;
  double l2;
  double l3;
  double ref_yaw_;
  double ref_pitch_;
  double ref_theta_;

  double start_theta_;
  double start_pitch_;
  double start_yaw_;
  double reset_theta_;
  double reset_pitch_;
  double reset_yaw_;

  void handle_goal(const geometry_msgs::msg::Pose::SharedPtr msg);
  void handle_start_motion(const std_msgs::msg::Empty::SharedPtr msg);
  void handle_reset_motion(const std_msgs::msg::Empty::SharedPtr msg);
  double solve_theta(double L1, double L2, double L3, double r);
};

}  // namespace my_trajectory_sender

#endif  // MY_TRAJECTORY_SENDER__JOINT_TRAJECTORY_PUBLISHER_HPP_
