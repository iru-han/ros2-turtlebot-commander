#include "my_turtle_gui/patrol_server.hpp"

using namespace std::placeholders;

PatrolServer::PatrolServer(): Node("turtlebot3_patrol_server") {
  action_server_ = rclcpp_action::create_server<Patrol>(
    this, "turtlebot3",
    std::bind(&PatrolServer::handle_goal, this, _1, _2),
    std::bind(&PatrolServer::handle_cancel, this, _1),
    std::bind(&PatrolServer::handle_accepted, this, _1)
  );

  safety_service_ = this->create_service<std_srvs::srv::SetBool>(
    "toggle_safety",
    std::bind(&PatrolServer::handle_safety_toggle, this, std::placeholders::_1, std::placeholders::_2)
  );

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&PatrolServer::odom_callback, this, _1)
  );

  RCLCPP_INFO(this->get_logger(), "PID Patrol Server Start!");
}

void PatrolServer::handle_safety_toggle(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response
) {
  is_safety_mode_ = request->data;

  response->success = true;
  response->message = is_safety_mode_ ? "Safety Mode ON" : "Safety Mode OFF";

  RCLCPP_INFO(this->get_logger(), "Service Request: %s", response->message.c_str());
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

// ==========================================================
// 1. 쿼터니언(4개 숫자)을 오일러 각도(Yaw)로 변환
// ==========================================================
double PatrolServer::get_yaw(const nav_msgs::msg::Odometry & odom) {
  // 쿼터니언 구조체 가져오기
  auto q = odom.pose.pose.orientation;

  RCLCPP_INFO(this->get_logger(), "Quat: w=%.2f, x=%.2f, y=%.2f, z=%.2f", q.w, q.x, q.y, q.z);

  // 공식: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  // atan2는 -PI ~ +PI (-180도 ~ +180도) 사이의 값을 반환합니다.r
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

  rclcpp::Rate loop_rate(50); // 50Hz 정밀 체크

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) return false;

    // double dx = current_odom_.pose.pose.position.x - start_x;
    // double dy = current_odom_.pose.pose.position.y - start_y;
    // double traveled_dist = std::sqrt(dx*dx + dy*dy);

    // 거리 계산 (피타고라스: a^2 + b^2 = c^2)
    double traveled_dist = calculate_distance(start_x,
      start_y,
      current_odom_.pose.pose.position.x,
      current_odom_.pose.pose.position.y);

    // 오차 계산
    double error = target_distance - traveled_dist;

    // P 제어: 거리가 많이 남으면 빠름, 적게 남으면 느림
    double speed = error * 1.0; // Kp = 1.0

    // 속도 제한 (너무 느리면 안 가니까 최소 0.05 보장, 최대 0.2 제한)
    if (speed > 0.2) speed = 0.2;
    if (speed < 0.05) speed = 0.05;

    if (error < 0.01) break; // 1cm 이내 도착 시 종료

    twist.linear.x = speed;
    cmd_vel_pub_->publish(twist);

    feedback->state = "Dist Error: " + std::to_string(error);
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  twist.linear.x = 0.0;
  cmd_vel_pub_->publish(twist);

  // 미끄러짐 안정화 대기
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  return true;
}

bool PatrolServer::rotate(double target_angle_deg, const std::shared_ptr<GoalHandlePatrol> goal_handle) {
  auto feedback = std::make_shared<Patrol::Feedback>();
  auto twist = geometry_msgs::msg::Twist();

  // 목표 각도를 라디안으로 변환 (deg * 3.14 / 180)
  double target_rad = target_angle_deg * M_PI / 180.0;

  // 현재 각도 저장 (시작점)
  double start_yaw = get_yaw(current_odom_);

  // PID 제어 변수 설정
  double Kp = 1.5; // P게인: 클수록 빨리 돔
  double Ki = 0.01; // I게인: 작을수록 미세 오차를 천천히 잡음 (너무 크면 흔들림)
  double Kd = 1.0; // D게인: 클수록 브레이크를 세게 밟음 (오버슈트 방지)

  double prev_error = 0.0; // 직전 오차 (D제어용)
  double integral_error = 0.0; // 누적 오차 (I제어용)

  // 50Hz (0.02초마다 제어)
  rclcpp::Rate loop_rate(50);
  double dt = 0.02;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) return false;

    // 1. 현재 각도 계산
    double current_yaw = get_yaw(current_odom_);

    // 2. 현재 회전한 양 계산 (현재 - 시작)
    double rotated_amount = current_yaw - start_yaw;

    // 각도 보정 (-PI ~ PI 넘어가면 반대편으로 계산되는 문제 해결)
    if (rotated_amount > M_PI) rotated_amount -= 2.0 * M_PI;
    else if (rotated_amount < -M_PI) rotated_amount += 2.0 * M_PI;

    // 3. 오차(Error) 계산: 목표값 - 현재값
    double error = target_rad - rotated_amount;

    // ★ PID 수식 적용 ★

    // P항: 오차에 비례 (멀면 빨리, 가까우면 느리게)
    double P_term = Kp * error;

    // I항: 오차 누적 (미세하게 안 닿을 때 밀어주기)
    integral_error += error * dt;
    double I_term = Ki * integral_error;

    // D항: 오차의 변화율 (기울기, 급격히 가까워지면 브레이크)
    double derivative = (error - prev_error) / dt;
    double D_term = Kd * derivative;

    // 최종 출력 (속도) = P + I + D
    double angular_velocity = P_term + I_term + D_term;

    // 다음 턴을 위해 현재 오차 저장
    prev_error = error;

    // 4. 안전장치: 너무 빠르면 위험하니까 최대 속도 제한 (Clamp)
    // std::clamp(값, 최소, 최대)
    double max_speed = 1.0;
    angular_velocity = std::clamp(angular_velocity, -max_speed, max_speed);

    // 5. 종료 조건: 오차가 매우 작으면(0.01 rad ≈ 0.5도) 멈춤
    if (std::abs(error) < 0.01) {
      break;
    }

    // 명령 전송
    twist.linear.x = 0.0;
    twist.angular.z = angular_velocity;
    cmd_vel_pub_->publish(twist);

    // 피드백 전송
    feedback->state = "PID Error: " + std::to_string(error);
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  // 완전 정지
  twist.angular.z = 0.0;
  cmd_vel_pub_->publish(twist);

  // 관성 때문에 조금 더 미끄러지는 시간을 기다려줍니다. (안정화)
  rclcpp::sleep_for(std::chrono::milliseconds(500));

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
