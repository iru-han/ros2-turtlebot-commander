#include "my_turtle_gui/patrol_server.hpp"

using namespace std::placeholders;

PatrolServer::PatrolServer(): Node("turtlebot3_patrol_server") {
  // 서비스 전용 콜백 그룹 생성 (데드락 방지)
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto srv_options = rcl_interfaces::msg::ParameterDescriptor(); // 임시

  action_server_ = rclcpp_action::create_server<Patrol>(
    this, "turtlebot3",
    std::bind(&PatrolServer::handle_goal, this, _1, _2),
    std::bind(&PatrolServer::handle_cancel, this, _1),
    std::bind(&PatrolServer::handle_accepted, this, _1)
  );

  // 서비스 등록 시 콜백 그룹 지정
  safety_service_ = this->create_service<std_srvs::srv::SetBool>(
    "toggle_safety",
    std::bind(&PatrolServer::handle_safety_toggle, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_
  );

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&PatrolServer::odom_callback, this, _1)
  );

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&PatrolServer::scan_callback, this, _1)
  );

  RCLCPP_INFO(this->get_logger(), "PID Patrol Server Start!");
}

// 스캔 콜백 구현
void PatrolServer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float min_val = 100.0;
    // 전방 30도 및 후방(인덱스 끝) 30도 체크
    for (size_t i = 0; i < 30; ++i) {
        if (msg->ranges[i] > 0.01 && msg->ranges[i] < min_val) min_val = msg->ranges[i];
    }
    for (size_t i = msg->ranges.size() - 30; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] > 0.01 && msg->ranges[i] < min_val) min_val = msg->ranges[i];
    }
    min_dist_ = min_val;
}

void PatrolServer::handle_safety_toggle(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response
) {
  // 경고 해결: 이 변수를 의도적으로 사용하지 않음을 컴파일러에게 알림
  (void)request;

  // is_safety_mode_ = request->data;
  // 클라이언트가 보낸 request->data를 무시하고 서버 내부 상태를 토글합니다.
  // (이게 가장 확실한 토글 방식입니다)
  is_safety_mode_ = !is_safety_mode_;

  response->success = true;
  response->message = is_safety_mode_ ? "안전 모드 ON" : "안전 모드 OFF";

  RCLCPP_INFO(this->get_logger(), "상태 변경: %s", response->message.c_str());
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

    // ★ 안전 모드 체크: 장애물이 0.3m 이내면 이동 중단 및 에러 처리
    if (is_safety_mode_ && min_dist_ < 0.3) {
            feedback->state = "일시 정지됨: 장애물 감지"; // 이 메시지가 GUI로 전달됨
            goal_handle->publish_feedback(feedback);

            twist.linear.x = 0.0;
            cmd_vel_pub_->publish(twist);
            loop_rate.sleep();
            continue; // 장애물이 치워질 때까지 아래 이동 로직을 건너뜀
    }

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

    feedback->state = "주행 중 (오차: " + std::to_string(error) + "m)";
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  twist.linear.x = 0.0;
  cmd_vel_pub_->publish(twist);

  // 미끄러짐 안정화 대기
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  return true;
}

// ★ 각도 정규화 함수 추가 (수학적으로 매우 중요!)
// -PI ~ PI 사이로 각도를 유지해줍니다.
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool PatrolServer::rotate(double target_angle_deg, const std::shared_ptr<GoalHandlePatrol> goal_handle) {
    auto feedback = std::make_shared<Patrol::Feedback>();
    auto twist = geometry_msgs::msg::Twist();

    double target_rad = target_angle_deg * M_PI / 180.0;
    double start_yaw = get_yaw(current_odom_);
    double goal_yaw = normalize_angle(start_yaw + target_rad); // 목표 절대 각도

    // PID 게인 조정 (물리적 한계 고려)
    double Kp = 1.2, Ki = 0.005, Kd = 0.5;
    double prev_error = 0.0, integral_error = 0.0;

    rclcpp::Rate loop_rate(50);
    auto start_time = this->now();

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) return false;

        double current_yaw = get_yaw(current_odom_);
        // ★ 수학 포인트: 현재 각도와 목표 각도의 최소 차이 계산
        double error = normalize_angle(goal_yaw - current_yaw);

        // 종료 조건 (0.02 rad 약 1.1도 정도로 여유를 줌)
        if (std::abs(error) < 0.02) break;

        // 타임아웃 설정 (10초 이상 회전 못하면 멈춤)
        if ((this->now() - start_time).seconds() > 10.0) return false;

        // PID 수식
        integral_error += error * 0.02;
        // Integral Windup 방지 (수학적 안전장치)
        integral_error = std::clamp(integral_error, -0.5, 0.5);

        double derivative = (error - prev_error) / 0.02;
        double output = (Kp * error) + (Ki * integral_error) + (Kd * derivative);
        prev_error = error;

        twist.angular.z = std::clamp(output, -1.0, 1.0);
        cmd_vel_pub_->publish(twist);

        feedback->state = "회전 중: 오차 " + std::to_string(error);
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

    int sides = (goal->goal.x == 2.0) ? 3 : 4;
    double angle = (goal->goal.x == 2.0) ? 120.0 : 90.0;
    double dist = goal->goal.y;

    for (int i = 0; i < sides; ++i) {
        // move_straight가 false를 반환(장애물 감지 등)하면
        if (!move_straight(dist, goal_handle)) {
            if (goal_handle->is_canceling()) {
                result->result = false;
                goal_handle->canceled(result); // 사용자가 취소한 경우
                RCLCPP_INFO(this->get_logger(), "Patrol Canceled");
            } else {
                result->result = false;
                goal_handle->abort(result);    // 장애물 등으로 서버가 중단한 경우 ★ (중요!)
                RCLCPP_WARN(this->get_logger(), "Patrol Aborted due to obstacle");
            }
            return; // 함수 종료
        }

        if (!rotate(angle, goal_handle)) {
            if (goal_handle->is_canceling()) {
                result->result = false;
                goal_handle->canceled(result); // 사용자가 취소한 경우
                RCLCPP_INFO(this->get_logger(), "Patrol Canceled");
            } else {
                result->result = false;
                goal_handle->abort(result);    // 장애물 등으로 서버가 중단한 경우 ★ (중요!)
                RCLCPP_WARN(this->get_logger(), "Patrol Aborted due to obstacle");
            }
            return;
        }
    }

    if (rclcpp::ok()) {
        result->result = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Patrol Success!");
    }
}

void PatrolServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
}
