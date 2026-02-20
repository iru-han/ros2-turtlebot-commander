#ifndef PATROL_SERVER_HPP_
#define PATROL_SERVER_HPP_

// ROS 2 및 표준 라이브러리 헤더
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" // Action 서버 구현을 위한 필수 헤더

// ROS 2 메시지/서비스/액션 타입 헤더
#include "geometry_msgs/msg/twist.hpp"      // 로봇 속도 제어 명령용
#include "nav_msgs/msg/odometry.hpp"        // 로봇 위치 및 자세 수신용
#include "sensor_msgs/msg/laser_scan.hpp"   // LiDAR 센서 데이터 수신용
#include "turtlebot3_msgs/action/patrol.hpp" // 순찰 액션 인터페이스
#include "std_srvs/srv/set_bool.hpp"        // 안전 모드 ON/OFF 서비스용

// C++ 수학 및 유틸리티 헤더
#include <cmath>      // 삼각함수(atan2 등) 계산용
#include <thread>     // 액션 실행을 위한 별도 스레드 생성용
#include <algorithm> // clamp(값 제한) 함수 사용용

/**
 * @class PatrolServer
 * @brief 터틀봇3의 실제 주행 로직과 센서 처리를 담당하는 서버 노드
 * 액션 서버를 통해 순찰 명령을 수행하며, PID 제어를 통해 정밀 주행을 구현
 */
class PatrolServer: public rclcpp::Node {
public:
    // 복잡한 액션 타입 이름을 짧게 별칭으로 정의
    using Patrol = turtlebot3_msgs::action::Patrol;
    using GoalHandlePatrol = rclcpp_action::ServerGoalHandle<Patrol>;

    PatrolServer(); // 생성자: 노드 초기화 및 통신 인터페이스 설정

private:
    // Action Server 콜백 함수
    // 1. 클라이언트의 목표(Goal) 요청을 수락할지 결정
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Patrol::Goal> goal);
    // 2. 클라이언트의 취소 요청 처리
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePatrol> goal_handle);
    // 3. 목표 수락 후 실제 실행 스레드 시작
    void handle_accepted(const std::shared_ptr<GoalHandlePatrol> goal_handle);

    /**
     * @brief 실제 순찰 주행이 일어나는 메인 루틴 (별도 스레드에서 동작)
     */
    void execute(const std::shared_ptr<GoalHandlePatrol> goal_handle);

    // Service Server
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr safety_service_; // 서비스 서버 객체
    bool is_safety_mode_ = true; // 서버 내부의 안전 모드 활성화 상태 변수

    /**
     * @brief 안전 모드 토글 요청을 처리하는 콜백 함수
     */
    void handle_safety_toggle(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response
    );

    // Topic Subscriber 콜백
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);   // 위치 정보 수신
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg); // 장애물 정보 수신

    // 수학 및 제어 함수
    /**
     * @brief 쿼터니언 자세 데이터를 Yaw(각도) 값으로 변환
     */
    double get_yaw(const nav_msgs::msg::Odometry & odom);

    /**
     * @brief 두 지점 간의 직선 거리를 계산 (피타고라스 정리)
     */
    double calculate_distance(double x1, double y1, double x2, double y2);

    /**
     * @brief PID 제어를 이용한 정밀 직선 주행 함수
     */
    bool move_straight(double target_distance, const std::shared_ptr<GoalHandlePatrol> goal_handle);

    /**
     * @brief PID 제어를 이용한 정밀 회전 함수
     */
    bool rotate(double target_angle_deg, const std::shared_ptr<GoalHandlePatrol> goal_handle);

    // ROS 통신 인터페이스 및 변수
    rclcpp_action::Server<Patrol>::SharedPtr action_server_;         // 액션 서버 객체
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; // 속도 명령 발행기
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;   // 위치 구독기
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // 센서 구독기

    nav_msgs::msg::Odometry current_odom_; // 현재 로봇의 최신 오돔 데이터 보관
    bool is_odom_received_ = false;       // 데이터 수신 여부 확인용

    /**
     * @brief 서비스와 다른 통신 간의 데드락(Deadlock)을 방지하기 위한 콜백 그룹
     */
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    float min_dist_ = 100.0; // 가장 가까운 장애물과의 거리 (m)
};

#endif // PATROL_SERVER_HPP_
