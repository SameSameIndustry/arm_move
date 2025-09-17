#include "arm_move/arm_trajectory.hpp"

#include <chrono>
#include <cmath>
#include <math.h>
#include <vector>

using namespace std::chrono_literals;
using arm_move::ArmTrajectory;

// TODO いろんなMotionを追加していくその時の角度はすべての関節の角度を指定するようにする
ArmTrajectory::ArmTrajectory(const rclcpp::NodeOptions &options)
    : Node("arm_trajectory", options)
{
  turn_table_position_pub_ = this->create_publisher<control_msgs::msg::MultiDOFCommand>(
      "/turn_table_position_controller/reference", 10);       // turn_table_position_controllerに送る
  hand_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hand_position_controller/commands", 10);                       // hand_position_controllerに送る
  hand_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hand_yaw_controller/commands", 10);
  hand_pitch_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hand_pitch_controller/commands", 10);
  hand_gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hand_gripper_controller/commands", 10);


  goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/arm_move/goal_pose", 10, std::bind(&ArmTrajectory::handle_goal, this, std::placeholders::_1));
  start_motion_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
      "/arm_move/start_motion", 10, std::bind(&ArmTrajectory::handle_start_motion, this, std::placeholders::_1));
  init_motion_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
      "/arm_move/reset_motion", 10, std::bind(&ArmTrajectory::handle_reset_motion, this, std::placeholders::_1));
  catch_motion_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
      "/arm_move/catch_motion", 10, std::bind(&ArmTrajectory::handle_catch_motion, this, std::placeholders::_1));
  set_catch_motion_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
      "/arm_move/set_catch_motion", 10, std::bind(&ArmTrajectory::handle_set_catch_motion, this, std::placeholders::_1));
  release_motion_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
      "/arm_move/release_motion", 10, std::bind(&ArmTrajectory::handle_release_motion, this, std::placeholders::_1));
    handle_up_motion_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
      "/arm_move/up_motion", 10, std::bind(&ArmTrajectory::handle_up_motion, this, std::placeholders::_1));
    handle_down_motion_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
      "/arm_move/down_motion", 10, std::bind(&ArmTrajectory::handle_down_motion, this, std::placeholders::_1));

  declare_parameter("turn_table_position_controller_joint_names", std::vector<std::string>{});
  declare_parameter("hand_position_controller_joint_names", std::vector<std::string>{});
  declare_parameter("hand_yaw_controller_joint_names", std::vector<std::string>{});
  declare_parameter("hand_pitch_controller_joint_names", std::vector<std::string>{});
  declare_parameter("hand_gripper_controller_joint_names", std::vector<std::string>{});
  declare_parameter("l1", 0.5);
  declare_parameter("l2", 0.5);
  declare_parameter("l3", 0.5);
  declare_parameter("start_theta", 1.57);
  declare_parameter("start_pitch", 0.0);
  declare_parameter("start_yaw", 0.0);
  declare_parameter("start_hand_yaw", 0.0);
  declare_parameter("start_hand_pitch", 0.0);
  declare_parameter("gripper_opening", 0.0);
  declare_parameter("gripper_closing", 1.57);
  declare_parameter("reset_theta", 0.785);
  declare_parameter("reset_pitch", 0.0);
  declare_parameter("reset_yaw", 0.0);
  declare_parameter("initial_left_radial_angle", 1.57f);
  declare_parameter("initial_right_radial_angle", 1.57f);
  declare_parameter("down_arm_pitch", 0.18f);
  declare_parameter("up_arm_pitch", 0.36f);
  
  turn_table_position_controller_joint_names = get_parameter("turn_table_position_controller_joint_names").as_string_array();
  hand_position_controller_joint_names = get_parameter("hand_position_controller_joint_names").as_string_array();
  hand_yaw_controller_joint_names = get_parameter("hand_yaw_controller_joint_names").as_string_array();
  hand_pitch_controller_joint_names = get_parameter("hand_pitch_controller_joint_names").as_string_array();
  hand_gripper_controller_joint_names = get_parameter("hand_gripper_controller_joint_names").as_string_array();
  
  l1 = get_parameter("l1").as_double();
  l2 = get_parameter("l2").as_double();
  l3 = get_parameter("l3").as_double();
  start_theta_ = get_parameter("start_theta").as_double();
  start_pitch_ = get_parameter("start_pitch").as_double();
  start_yaw_ = get_parameter("start_yaw").as_double();
  start_hand_yaw_ = get_parameter("start_hand_yaw").as_double();
  start_hand_pitch_ = get_parameter("start_hand_pitch").as_double();
  reset_theta_ = get_parameter("reset_theta").as_double();
  reset_pitch_ = get_parameter("reset_pitch").as_double();
  reset_yaw_ = get_parameter("reset_yaw").as_double();
  gripper_opening_ = get_parameter("gripper_opening").as_double();
  gripper_closing_ = get_parameter("gripper_closing").as_double();
  initial_left_radial_angle_ = get_parameter("initial_left_radial_angle").as_double();
  initial_right_radial_angle_ = get_parameter("initial_right_radial_angle").as_double();
  down_arm_pitch_ = get_parameter("down_arm_pitch").as_double();
  up_arm_pitch_ = get_parameter("up_arm_pitch").as_double();

  ref_theta_ = start_theta_;
  ref_pitch_ = start_pitch_;
  ref_yaw_ = start_yaw_;
  ref_hand_yaw_ = start_hand_yaw_;
  ref_hand_pitch_ = start_hand_pitch_;
  ref_gripper_ = gripper_opening_;


  timer_ = this->create_wall_timer(1s, std::bind(&ArmTrajectory::timer_callback, this)); // 本当はactionにする
  start_time_ = this->get_clock()->now();
}

void ArmTrajectory::timer_callback()
{
  auto now = this->get_clock()->now();
  double elapsed = (now - start_time_).seconds();

  // ハンドの位置に関連するものをPositionController使用する前提で送る(odrive用)
  std_msgs::msg::Float64MultiArray hand_position_msg;
  hand_position_msg.data = {ref_theta_ - initial_left_radial_angle_, -(ref_theta_ - initial_right_radial_angle_)}; // 左右の順番(初期の角度でいい感じにする)

  // ターンテーブルの位置に関連するものをPIDControllerを使用する前提で送る(robo)
  control_msgs::msg::MultiDOFCommand turn_table_pid_msg;
  turn_table_pid_msg.dof_names = turn_table_position_controller_joint_names;
  turn_table_pid_msg.values = {ref_pitch_, ref_pitch_, ref_yaw_};
  // turn_table_pid_msg.values_dot = {0,0,0};

  // ハンドの先端のピッチ
  std_msgs::msg::Float64MultiArray hand_pitch_msg;
  hand_pitch_msg.data = {ref_hand_pitch_};

  // ハンドの先端のヨー
  std_msgs::msg::Float64MultiArray hand_yaw_msg;
  hand_yaw_msg.data = {ref_hand_yaw_};

  // ハンドのグリッパー
  std_msgs::msg::Float64MultiArray hand_gripper_msg;
  hand_gripper_msg.data = {ref_gripper_};

  turn_table_position_pub_->publish(turn_table_pid_msg);
  hand_position_pub_->publish(hand_position_msg);
  hand_pitch_pub_->publish(hand_pitch_msg);
  hand_yaw_pub_->publish(hand_yaw_msg);
  hand_gripper_pub_->publish(hand_gripper_msg);
}

//  TODO 要修正
void ArmTrajectory::handle_goal(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::Point ref_position = msg->pose.position;
  geometry_msgs::msg::Quaternion ref_orientation = msg->pose.orientation;
  double dx = ref_position.x;
  double dy = ref_position.y;
  // double dz = ref_position.z;
  double ref_radius = std::hypot(dx, dy); // xy平面での距離->z^2もいるかも
  ref_radius = ref_radius / std::cos(down_arm_pitch_);
  ref_theta_ = solve_theta(l1, l2, l3, ref_radius);
  // ref_pitch_ = std::asin(dz / ref_radius);
  ref_yaw_ = std::atan2(dy, dx) - M_PI/ 2.0; // 90度ずらす
}

void ArmTrajectory::handle_start_motion(
    const std_msgs::msg::Empty::SharedPtr msg)
{
  ref_theta_ = start_theta_;
  ref_pitch_ = start_pitch_;
  ref_yaw_ = start_yaw_;
  ref_hand_yaw_ = 0.0;
  ref_hand_pitch_ = 0.0;
  ref_gripper_ = gripper_opening_;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start motion received");
}

void ArmTrajectory::handle_reset_motion(
    const std_msgs::msg::Empty::SharedPtr msg)
{
  ref_theta_ = reset_theta_;
  ref_pitch_ = reset_pitch_;
  ref_yaw_ = reset_yaw_;
  ref_hand_yaw_ = 0.0;
  ref_hand_pitch_ = 0.0;
  ref_gripper_ = gripper_opening_;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init motion received");
}

void ArmTrajectory::handle_up_motion(
    const std_msgs::msg::Empty::SharedPtr msg)
{
  ref_pitch_ = up_arm_pitch_;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Up hand motion received");
}
void ArmTrajectory::handle_down_motion(
    const std_msgs::msg::Empty::SharedPtr msg)
{
  ref_pitch_ = down_arm_pitch_;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Down hand motion received");
}

void ArmTrajectory::handle_catch_motion(
    const std_msgs::msg::Empty::SharedPtr msg)
{
  ref_gripper_ = gripper_closing_;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Catch motion received");
}

void ArmTrajectory::handle_set_catch_motion(
    const std_msgs::msg::Empty::SharedPtr msg)
{
  ref_hand_yaw_ = 0.0;
  ref_hand_pitch_ = 1.57;
  ref_gripper_ = 0.0;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set catch motion received");
}

void ArmTrajectory::handle_release_motion(
    const std_msgs::msg::Empty::SharedPtr msg)
{
  ref_gripper_ = gripper_opening_;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Release motion received");
}

double ArmTrajectory::solve_theta(double L1, double L2, double L3, double r)
{
    // --- 入力値のチェック ---
    if (L1 <= 0.0 || L2 <= 0.0 || L3 <= 0.0 || r <= 0.0) {
        // エラーログなどを出すとより親切
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 1. モーター軸とアーム先端との直線距離を計算
    const double dist_motor_to_end = std::hypot(L1/2, r);

    // 2. 物理的に到達可能かチェック
    // L2とL3を一直線に伸ばした距離より遠い、または折りたたんだ距離より近い場合は到達不可
    if (dist_motor_to_end > L2 + L3 || dist_motor_to_end < std::abs(L2 - L3)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 3. 余弦定理を用いて各角度を計算
    
    // モーター軸、アーム先端、中間関節でなす三角形を考える
    // モーター軸の位置にある内角 (alpha)
    const double cos_alpha = (L2 * L2 + dist_motor_to_end * dist_motor_to_end - L3 * L3) / (2.0 * L2 * dist_motor_to_end);
    const double alpha = std::acos(std::clamp(cos_alpha, -1.0, 1.0));

    // ヨー軸中心、モーター軸、アーム先端でなす直角三角形を考える
    // ヨー軸中心の位置にある内角 (beta)
    const double beta = std::atan2(r, L1/2);

    // 4. 2つの解（エルボアップ/ダウン）を計算
    // モーターの回転方向やリンクの組み方によって、+alpha と -alpha のどちらを採用するかが決まる
    double theta_elbow_up = beta + alpha;   // 解1

    // -piからpiに正規化する
    const double two_pi = 2.0 * M_PI;
    if (theta_elbow_up > M_PI) {
        theta_elbow_up -= two_pi;
    } else if (theta_elbow_up < -M_PI) {
        theta_elbow_up += two_pi;
    }

    // ここでは片方の解（エルボアップ）を返すことにする
    // 必要に応じて、どちらの解が適切かを選択するロジックを追加してください
    return theta_elbow_up;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(arm_move::ArmTrajectory)
// This code is part of a ROS 2 package that publishes a trajectory message for an arm robot.
