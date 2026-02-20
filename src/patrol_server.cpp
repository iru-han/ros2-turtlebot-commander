#include "my_turtle_gui/patrol_server.hpp"

using namespace std::placeholders;

/**
 * 생성자: Patrol 서버 노드 초기화
 * 액션 서버, 서비스 서버, 토픽 발행/구독을 모두 설정합니다.
 */
PatrolServer::PatrolServer(): Node("turtlebot3_patrol_server") {
  // 1. 데드락 방지: 서비스 전용 콜백 그룹 생성
  // 서비스 응답 중에 다른 콜백(액션 등)이 멈추지 않도록 독립적인 실행 환경을 제공함
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto srv_options = rcl_interfaces::msg::ParameterDescriptor(); // 임시

  // 2. Action Server: "turtlebot3"이라는 이름의 액션 서버 생성
  // 목표 수락, 취소 처리, 실행 시작에 대한 각각의 핸들러 함수를 연결함
  action_server_ = rclcpp_action::create_server<Patrol>(
    this, "turtlebot3",
    std::bind(&PatrolServer::handle_goal, this, _1, _2),
    std::bind(&PatrolServer::handle_cancel, this, _1),
    std::bind(&PatrolServer::handle_accepted, this, _1)
  );

  // 3. Service Server: 안전 모드 토글 서비스 생성
  // 위에서 만든 cb_group_을 지정하여 안전하게 동작하도록 함
  safety_service_ = this->create_service<std_srvs::srv::SetBool>(
    "toggle_safety",
    std::bind(&PatrolServer::handle_safety_toggle, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    cb_group_
  );

  // 4. Topic Publisher: 로봇 바퀴 제어를 위한 속도 명령 발행기
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 5. Topic Subscriber: 로봇 위치(Odom) 및 레이저 센서(Scan) 구독
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&PatrolServer::odom_callback, this, _1)
  );

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&PatrolServer::scan_callback, this, _1)
  );

  RCLCPP_INFO(this->get_logger(), "PID 기반 패트롤 서버 시작!");
}

/**
 * Scan 콜백: LiDAR 센서 데이터를 분석하여 가장 가까운 장애물 거리를 업데이트
 */
void PatrolServer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float min_val = 100.0;

    // 전방 기준 왼쪽 30도(인덱스 0~29)와 오른쪽 30도(인덱스 끝쪽)를 합쳐 총 60도 감시
    for (size_t i = 0; i < 30; ++i) {
        if (msg->ranges[i] > 0.01 && msg->ranges[i] < min_val) min_val = msg->ranges[i];
    }
    for (size_t i = msg->ranges.size() - 30; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] > 0.01 && msg->ranges[i] < min_val) min_val = msg->ranges[i];
    }
    min_dist_ = min_val; // 클래스 변수에 최소 거리 저장
}

/**
 * Service 콜백: 안전 모드 ON/OFF 요청 처리
 */
void PatrolServer::handle_safety_toggle(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response
) {
  (void)request; // 매개변수 미사용 경고 방지

  // 서버 내부의 안전 모드 상태를 반전(Toggle)시킴
  is_safety_mode_ = !is_safety_mode_;

  response->success = true;
  response->message = is_safety_mode_ ? "안전 모드 ON" : "안전 모드 OFF";

  RCLCPP_INFO(this->get_logger(), "상태 변경: %s", response->message.c_str());
}

/*
* 클라이언트가 목표를 보냈을 때 (Accept/Reject 결정)
*/
rclcpp_action::GoalResponse PatrolServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Patrol::Goal> goal
) {
  (void) uuid;
  RCLCPP_INFO(this->get_logger(), "순찰 목표 수신: 모드 %f", goal->goal.x);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/*
* 클라이언트가 취소 요청을 보냈을 때
*/
rclcpp_action::CancelResponse PatrolServer::handle_cancel(
  const std::shared_ptr<GoalHandlePatrol> goal_handle
) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "순찰 취소 요청 수신");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/*
* 목표가 수락되었을 때 (실제 실행 스레드 분리)
*/
void PatrolServer::handle_accepted(
  const std::shared_ptr<GoalHandlePatrol> goal_handle)
{
  // 메인 스레드가 멈추지 않도록 새로운 스레드를 생성하여 실제 순찰 로직(execute)을 실행
  std::thread{std::bind(&PatrolServer::execute, this, _1), goal_handle}.detach();
}

/**
 * 쿼터니언(Quaternion)을 사람이 이해하기 쉬운 각도(Yaw)로 변환
 */
double PatrolServer::get_yaw(const nav_msgs::msg::Odometry & odom) {
  // 쿼터니언 구조체 가져오기
  auto q = odom.pose.pose.orientation;

  RCLCPP_INFO(this->get_logger(), "Quat: w=%.2f, x=%.2f, y=%.2f, z=%.2f", q.w, q.x, q.y, q.z);

  // 공식: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  // atan2는 -PI ~ +PI (-180도 ~ +180도) 사이의 값을 반환
  return std::atan2(siny_cosp, cosy_cosp);
}

/**
 * 수학: 각도를 -PI ~ PI 범위 안으로 정규화
 * 로봇이 179도에서 181도로 돌 때 -179도로 인식되어 엉뚱하게 회전하는 것을 방지함
 */
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * 수학: 두 지점 사이의 직선 거리를 계산 (피타고라스 정리)
 */
double PatrolServer::calculate_distance(double x1, double y1, double x2, double y2) {
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

/**
 * 주행: 정밀 직선 이동 함수
 * 오차에 비례하여 속도를 조절하는 P 제어 적용
 */
bool PatrolServer::move_straight(double target_distance, const std::shared_ptr<GoalHandlePatrol> goal_handle) {
  auto feedback = std::make_shared<Patrol::Feedback>();
  auto twist = geometry_msgs::msg::Twist();

  double start_x = current_odom_.pose.pose.position.x;
  double start_y = current_odom_.pose.pose.position.y;

  rclcpp::Rate loop_rate(50); // 1초에 50번 실행 (정밀도 유지)

  while (rclcpp::ok()) {
    // GUI에서 정지 버튼을 누르면 즉시 중단
    if (goal_handle->is_canceling()) return false;

    // 안전 모드 체크: 장애물이 0.3m 이내면 이동 중단 및 에러 처리
    if (is_safety_mode_ && min_dist_ < 0.3) {
            feedback->state = "일시 정지됨: 장애물 감지";
            goal_handle->publish_feedback(feedback);

            twist.linear.x = 0.0;
            cmd_vel_pub_->publish(twist);
            loop_rate.sleep();
            continue; // 장애물이 치워질 때까지 아래 이동 로직을 건너뜀
    }

    // 이동한 거리 계산 (피타고라스: a^2 + b^2 = c^2)
    double traveled_dist = calculate_distance(start_x,
      start_y,
      current_odom_.pose.pose.position.x,
      current_odom_.pose.pose.position.y);

    // 오차 계산
    double error = target_distance - traveled_dist;

    // P 제어: 목표에 가까워질수록 천천히 주행하여 정밀하게 정지함
    double speed = error * 1.0; // Kp = 1.0

    // 속도 제한 (너무 느리면 안 가니까 최소 0.05 보장, 최대 0.2 제한)
    if (speed > 0.2) speed = 0.2;
    if (speed < 0.05) speed = 0.05;

    if (error < 0.01) break;// 1cm 오차 이내면 목표 도달로 간주하고 종료

    twist.linear.x = speed;
    cmd_vel_pub_->publish(twist);

    // GUI 리스트 위젯에 표시될 상태 전송
    feedback->state = "주행 중 (오차: " + std::to_string(error) + "m)";
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  twist.linear.x = 0.0;
  cmd_vel_pub_->publish(twist);
  rclcpp::sleep_for(std::chrono::milliseconds(500)); // 관성 안정화를 위한 대기

  return true;
}

/**
 * 주행: 정밀 회전 함수 (PID 제어 적용)
 */
bool PatrolServer::rotate(double target_angle_deg, const std::shared_ptr<GoalHandlePatrol> goal_handle) {
    auto feedback = std::make_shared<Patrol::Feedback>();
    auto twist = geometry_msgs::msg::Twist();

    double target_rad = target_angle_deg * M_PI / 180.0; // 도 단위를 라디안으로 변환
    double start_yaw = get_yaw(current_odom_);
    double goal_yaw = normalize_angle(start_yaw + target_rad); // 최종 목표 절대 각도

    // PID 제어 계수 (로봇 마찰 및 관성 고려)
    double Kp = 1.2, Ki = 0.005, Kd = 0.5;
    double prev_error = 0.0, integral_error = 0.0;

    rclcpp::Rate loop_rate(50);
    auto start_time = this->now();

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) return false;

        double current_yaw = get_yaw(current_odom_);
        // 현재 각도와 목표 각도 사이의 최단 오차 계산
        double error = normalize_angle(goal_yaw - current_yaw);

        // 오차가 0.02 rad(약 1.1도) 이내면 회전 종료
        if (std::abs(error) < 0.02) break;

        // 예외 처리: 10초 이상 회전이 안 되면(끼임 등) 실패 처리
        if ((this->now() - start_time).seconds() > 10.0) return false;

        // PID 계산 로직
        integral_error += error * 0.02; // 오차 누적 (I)
        integral_error = std::clamp(integral_error, -0.5, 0.5); // 과도한 누적 방지(Anti-Windup)

        double derivative = (error - prev_error) / 0.02; // 오차 변화율 (D)
        double output = (Kp * error) + (Ki * integral_error) + (Kd * derivative);
        prev_error = error;

        twist.angular.z = std::clamp(output, -1.0, 1.0); // 최대 회전 속도 제한
        cmd_vel_pub_->publish(twist);

        feedback->state = "회전 중: 오차 " + std::to_string(error);
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

  // 완전 정지
  twist.angular.z = 0.0;
  cmd_vel_pub_->publish(twist);

  // 관성 때문에 조금 더 미끄러지는 시간을 기다림. (안정화)
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  return true;
}

/**
 * 실행 메인 로직: 액션 목표가 수락되면 실제로 사각형/삼각형 순찰을 도는 루프
 */
void PatrolServer::execute(const std::shared_ptr<GoalHandlePatrol> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Patrol::Result>();

    // 모드 설정 (1.0: 사각형=4변, 2.0: 삼각형=3변)
    int sides = (goal->goal.x == 2.0) ? 3 : 4;
    double angle = (goal->goal.x == 2.0) ? 120.0 : 90.0;
    double dist = goal->goal.y;

    for (int i = 0; i < sides; ++i) {
        // 1. 직선 이동 시도
        if (!move_straight(dist, goal_handle)) {
            if (goal_handle->is_canceling()) {
                result->result = false;
                goal_handle->canceled(result); // 사용자가 취소한 경우
                RCLCPP_INFO(this->get_logger(), "Patrol Canceled");
            } else {
                result->result = false;
                goal_handle->abort(result); // 장애물 등에 의한 강제 중단
                RCLCPP_WARN(this->get_logger(), "Patrol Aborted due to obstacle");
            }
            return; // 함수 종료
        }

        // 2. 회전 시도
        if (!rotate(angle, goal_handle)) {
            if (goal_handle->is_canceling()) {
                result->result = false;
                goal_handle->canceled(result); // 사용자가 취소한 경우
                RCLCPP_INFO(this->get_logger(), "Patrol Canceled");
            } else {
                result->result = false;
                goal_handle->abort(result); // 장애물 등에 의한 강제 중단
                RCLCPP_WARN(this->get_logger(), "Patrol Aborted due to obstacle");
            }
            return;
        }
    }

    // 모든 경로를 무사히 돌면 성공 결과 반환
    if (rclcpp::ok()) {
        result->result = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "모든 순찰 경로를 성공적으로 완주");
    }
}

/**
 * Odom 콜백: 현재 로봇의 위치 데이터를 최신화함
 */
void PatrolServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
}
