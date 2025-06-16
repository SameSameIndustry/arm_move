#include "arm_move/arm_trajectory.hpp"

#include <chrono>
#include <cmath>
#include <math.h>
#include <vector>

using namespace std::chrono_literals;
using arm_move::ArmTrajectory;

ArmTrajectory::ArmTrajectory(const rclcpp::NodeOptions & options)
: Node("arm_trajectory", options)
{
  publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_position_controller/joint_trajectory", 10); //joint_trajectory_position_controllerに送る
   subscriber_= this->create_subscription<geometry_msgs::msg::Pose>(// TODO:本来アクションにしてそのアクション内でpub --onceするようにする
    "/arm_move/goal_radius", 10,std::bind(&ArmTrajectory::handle_goal, this, std::placeholders::_1));
  declare_parameter("joint_names", std::vector<std::string>{});
  declare_parameter("l1", 0.5);
  declare_parameter("l2", 0.5);
  declare_parameter("l3", 0.5);
  joint_names_ = get_parameter("joint_names").as_string_array();
  l1 = get_parameter("l1").as_double();
  l2 = get_parameter("l2").as_double();
  l3 = get_parameter("l3").as_double();
  ref_theta_ = 0.0; // 初期値を設定

  timer_ = this->create_wall_timer(1s, std::bind(&ArmTrajectory::timer_callback, this));
  start_time_ = this->get_clock()->now();
}

void ArmTrajectory::timer_callback()
{
  auto now = this->get_clock()->now();
  double elapsed = (now - start_time_).seconds();

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {
    ref_theta_,
    -ref_theta_,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
  };
  point.time_from_start = rclcpp::Duration::from_seconds(1.0);
  traj_msg.points.push_back(point);

  publisher_->publish(traj_msg);
}

void ArmTrajectory::handle_goal(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  geometry_msgs::msg::Point ref_position = msg->position;
  geometry_msgs::msg::Quaternion ref_orientation = msg->orientation;
  double dx = ref_position.x;
  double dy = ref_position.y;
  double ref_radius = std::hypot(dx, dy);
  ref_theta_ = solve_theta(l1,l2,l3,ref_radius);
}

double ArmTrajectory::solve_theta(double L1, double L2, double L3, double r)
{
    // --- 入力チェック ---
    if (L1 <= 0.0 || L2 <= 0.0 || L3 <= 0.0 || r <= 0.0)
        return std::numeric_limits<double>::quiet_NaN();

    // --- 定数計算 ---
    const double denom = std::sqrt(L1 * L1 + r * r);
    const double K =
        (L1 * L1 + r * r - L3 * L3 + L2 * L2) / (2.0 * L2);

    // --- 定義域チェック ---
    const double ratio = K / denom;
    if (std::fabs(ratio) > 1.0)
        return std::numeric_limits<double>::quiet_NaN(); // 作図不可能

    // --- θ1 を計算 ---
    const double theta =
        std::asin(std::clamp(ratio, -1.0, 1.0)) - std::atan(L1 / r);

    // （必要なら 0〜2π に正規化するなど適宜処理）
    return theta; // rad
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(arm_move::ArmTrajectory)
// This code is part of a ROS 2 package that publishes a trajectory message for an arm robot.

