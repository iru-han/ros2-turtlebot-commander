#include "my_turtle_gui/patrol_server.hpp"

using namespace std::placeholders;

PatrolServer::PatrolServer(): Node("turtlebot3_patrol_server") {
  action_server_ = rclcpp_action::create_server<Patrol>(
    this, "turtlebot3",
    std::bind(&PatrolServer::handle_goal, this, _1, _2),
    std::bind(&PatrolServer::handle_cancel, this, _1),
    std::bind(&PatrolServer::handle_accepted, this, _1)
  );

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&PatrolServer::odom_callback, this, _1)
  );

  RCLCPP_INFO(this->get_logger(), "Patrol Server Start!");
}

// 1. 클라이언트가 목표를 보냈을 때 (Accept/Reject 결정)
rclcpp_action::GoalResponse PatrolServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Patrol::Goal> goal
) {
  (void) uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request: mode %f", goal->goal.x);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 2. 클라이언트가 취소 요청을 보냈을 때
rclcpp_action::CancelResponse PatrolServer::handle_cancel(
  const std::shared_ptr<GoalHandlePatrol> goal_handle
) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 3. 목표가 수락되었을 때 (실제 실행 스레드 분리)
void PatrolServer::handle_accepted(
  const std::shared_ptr<GoalHandlePatrol> goal_handle)
{
  std::thread{std::bind(&PatrolServer::execute, this, _1), goal_handle}.detach();
}

// ★ 쿼터니언 -> Yaw (radian) 변환 수식
double PatrolServer::get_yaw(const nav_msgs::msg::Odometry & odom) {
  auto q = odom.pose.pose.orientation;

  RCLCPP_INFO(this->get_logger(), "Quat: w=%.2f, x=%.2f, y=%.2f, z=%.2f", q.w, q.x, q.y, q.z);

  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// ★ 두 점 사이의 거리 계산 (피타고라스)
double PatrolServer::calculate_distance(double x1, double y1, double x2, double y2) {
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// ★ 정밀 직진 함수 (오돔 기반)
bool PatrolServer::move_straight(double target_distance, const std::shared_ptr<GoalHandlePatrol> goal_handle) {
  auto feedback = std::make_shared<Patrol::Feedback>();
  auto twist = geometry_msgs::msg::Twist();

  double start_x = current_odom_.pose.pose.position.x;
  double start_y = current_odom_.pose.pose.position.y;
  double traveled_dist = 0.0;

  rclcpp::Rate loop_rate(20); // 20Hz

  while (traveled_dist < target_distance) {
    if (goal_handle->is_canceling()) return false;
    traveled_dist = calculate_distance(start_x, start_y,
    current_odom_.pose.pose.position.x,
    current_odom_.pose.pose.position.y);

    twist.linear.x = 0.15; // forward speed
    cmd_vel_pub_->publish(twist);

    feedback->state = "Moving: " + std::to_string(traveled_dist) + "m";
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  twist.linear.x = 0.0;
  cmd_vel_pub_->publish(twist);
  return true;
}

bool PatrolServer::rotate(double target_angle_deg, const std::shared_ptr<GoalHandlePatrol> goal_handle) {
  auto feedback = std::make_shared<Patrol::Feedback>();
  auto twist = geometry_msgs::msg::Twist();

  double target_rad = target_angle_deg * M_PI / 180.0;
  double start_yaw = get_yaw(current_odom_);
  double rotated_yaw = 0.0;

  rclcpp::Rate loop_rate(20);

  while (std::abs(rotated_yaw) < std::abs(target_rad)) {
    if (goal_handle->is_canceling()) return false;

    double current_yaw = get_yaw(current_odom_);
    rotated_yaw = current_yaw - start_yaw;

    // 각도 보정 (-PI ~ PI 범위 처리)
    if (rotated_yaw > M_PI) rotated_yaw -= 2.0 * M_PI;
    else if (rotated_yaw < -M_PI) rotated_yaw += 2.0 * M_PI;

    twist.angular.z = (target_angle_deg > 0) ? 0.3 : -0.3; // 회전 방향 설정
    cmd_vel_pub_->publish(twist);

    feedback->state = "rotating.. " + std::to_string(rotated_yaw * 180.0 / M_PI) + "degree";
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  twist.angular.z = 0.0;
  cmd_vel_pub_->publish(twist);
  return true;
}

void PatrolServer::execute(const std::shared_ptr<GoalHandlePatrol> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Patrol::Result>();

  // 1=사각형(90도씩 4번), 2=삼각형(120도씩 3번)
  int sides = (goal->goal.x == 2.0) ? 3 : 4; // 2=삼각형, 그외 사각형
  double angle = (goal->goal.x == 2.0) ? 120.0 : 90.0;
  double dist = goal->goal.y;

  RCLCPP_INFO(this->get_logger(), "Patrol Start! Mode: %f", goal->goal.x);

  for (int i = 0; i < sides; ++i) {
    // straight and rotate
    if (!move_straight(dist, goal_handle) || !rotate(angle, goal_handle)) {
      result->result = false;
      goal_handle->canceled(result);
      return;
    }
  }

  result->result = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Patrol successfully completed!");
}

void PatrolServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
}
