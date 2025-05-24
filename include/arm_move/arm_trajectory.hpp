#ifndef MY_TRAJECTORY_SENDER__ARM_TRAJECTORY_HPP_
#define MY_TRAJECTORY_SENDER__ARM_TRAJECTORY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64.hpp"

namespace arm_move
{

class ArmTrajectory : public rclcpp::Node
{
public:
  ArmTrajectory(const rclcpp::NodeOptions & options);

private:
  void timer_callback();

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_; // TODO:本来ゴーをもらう型にする
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  std::string joint_left_name_;
  std::string joint_right_name_;
  std::string joint_pitch_name_;
  double l1;
  double l2;
  double l3;
  double ref_theta_;
  void handle_goal(const std_msgs::msg::Float64::SharedPtr msg); // 仮で半径のみできるか確かめる
  double solveTheta1(double L1, double L2, double L3, double r);
};

}  // namespace my_trajectory_sender

#endif  // MY_TRAJECTORY_SENDER__JOINT_TRAJECTORY_PUBLISHER_HPP_
