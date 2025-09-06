#ifndef MY_TRAJECTORY_SENDER__ARM_TRAJECTORY_HPP_
#define MY_TRAJECTORY_SENDER__ARM_TRAJECTORY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "control_msgs/msg/multi_dof_command.hpp"
#include "std_msgs/msg/empty.hpp"

namespace arm_move
{

class ArmTrajectory : public rclcpp::Node
{
public:
  ArmTrajectory(const rclcpp::NodeOptions & options);

private:
  void timer_callback();

  // Publisher
  rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr turn_table_position_pub_; // 全部ロボマス
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hand_position_pub_; // 全部オードライブ
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hand_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hand_pitch_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hand_gripper_pub_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_pose_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_motion_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr init_motion_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr catch_motion_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr set_catch_motion_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr release_motion_subscriber_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  // パラメータから取得する変数
  std::vector<std::string> hand_position_controller_joint_names;
  std::vector<std::string> turn_table_position_controller_joint_names;
  std::vector<std::string> hand_yaw_controller_joint_names;
  std::vector<std::string> hand_pitch_controller_joint_names;
  std::vector<std::string> hand_gripper_controller_joint_names; 
  double l1;
  double l2;
  double l3;
  double start_theta_;
  double start_pitch_;
  double start_yaw_;
  double reset_theta_;
  double reset_pitch_;
  double reset_yaw_;
  double gripper_opening_;
  double gripper_closing_;
  double start_hand_yaw_;
  double start_hand_pitch_;
  double initial_left_radial_angle_;
  double initial_right_radial_angle_;
  double initial_left_pitch_angle_;
  double initial_right_pitch_angle_;

  double ref_yaw_;
  double ref_pitch_;
  double ref_theta_;
  double ref_hand_yaw_;
  double ref_hand_pitch_;
  double ref_gripper_;

  void handle_goal(const geometry_msgs::msg::Pose::SharedPtr msg);
  // いろんなモーション
  void handle_start_motion(const std_msgs::msg::Empty::SharedPtr msg);
  void handle_reset_motion(const std_msgs::msg::Empty::SharedPtr msg);
  void handle_catch_motion(const std_msgs::msg::Empty::SharedPtr msg);
  void handle_set_catch_motion(const std_msgs::msg::Empty::SharedPtr msg);
  void handle_release_motion(const std_msgs::msg::Empty::SharedPtr msg);

  double solve_theta(double L1, double L2, double L3, double r);
};

}  // namespace my_trajectory_sender

#endif  // MY_TRAJECTORY_SENDER__JOINT_TRAJECTORY_PUBLISHER_HPP_
