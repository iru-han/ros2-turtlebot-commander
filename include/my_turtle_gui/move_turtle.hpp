#ifndef MOVE_TURTLE_HPP_
#define MOVE_TURTLE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" // action required
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlebot3_msgs/action/patrol.hpp" // patrol action
#include "std_srvs/srv/set_bool.hpp"

#include <QString>
#include <functional>
#include <cmath>

class Move_turtle : public rclcpp::Node {
public:
    using Patrol = turtlebot3_msgs::action::Patrol;
    using GoalHandlePatrol = rclcpp_action::ClientGoalHandle<Patrol>;

    Move_turtle() : Node("move_turtle_node"), velocity(0.0), angular(0.0), is_safety_on(true) {
        // 1. [Topic Pub] 수동 조종
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 2. [Topic Sub] 오돔 (위치) - QoS Reliable
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>{
            "/odom", qos_reliable, std::bind(&Move_turtle::odom_callback, this, std::placeholders::_1)
        };

        // 3. [Topic Sub] 스캔 (장애물) - QoS SensorData
        auto qos_sensor = rclcpp::SensorDataQoS();
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_sensor, std::bind(&Move_turtle::scan_callback, this, std::placeholders::_1)
        );

        // 4. [Service Server] 안전 모드 토글 (과제 요건 충족용!)
        srv_safety_ = this->create_service<std_srvs::srv::SetBool>(
            "/toggle_safety", std::bind(&Move_turtle::safety_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // 5. [Action Client] 순찰 명령
        action_client_ = rclcpp_action::create_client<Patrol>(this, "turtlebot3");

        // 타이머 (수동 조종용)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Move_turtle::timer_callback, this));
    }

    double velocity;
    double angular;
    bool is_safety_on;

    // GUI로 글자를 보내주기 위한 콜백 함수 (파이썬의 addItem 역할 대체)
    std::function<void(QString)> log_callback;
    std::function<void(double, double, double)> pose_callback; // x, y, theta
    std::function<void(bool)> warning_callback;

    // ==========================================
    // [Action] 순찰 명령 보내기 함수
    // ==========================================
    void send_patrol_goal(int mode) { // 1: square, 2: triangle
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            if(log_callback) log_callback("액션 서버를 찾을 수 없습니다! (서버 켜져있나요?)");
            return;
        }

        auto goal_msg = Patrol::Goal();
        goal_msg.goal.x = (float)mode; // 모드
        goal_msg.goal.y = 1.0;         // 거리 1m
        goal_msg.goal.z = 1.0;         // 횟수 1회

        auto send_goal_options = rclcpp_action::Client<Patrol>::SendGoalOptions();

        // 피드백(중간보고) 받으면 실행할 함수 연결
        send_goal_options.feedback_callback = [this](
            GoalHandlePatrol::SharedPtr,
            const std::shared_ptr<const Patrol::Feedback> feedback) {
                if(log_callback) log_callback(QString("순찰 중: %1").arg(feedback->state.c_str()));
            };

        // 결과 받으면 실행할 함수 연결
        send_goal_options.result_callback = [this](const GoalHandlePatrol::WrappedResult & result) {
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
                if(log_callback) log_callback("순찰 완료!");
        };

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
        if(log_callback) log_callback("순찰 명령 전송함!");
    }

private:
    // [Topic] 수동 조종 타이머
    void timer_callback() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = velocity;
        msg.angular.z = angular;
        pub_cmd_->publish(msg);

        // GUI에 로그를 찍기 위해 콜백 실행
        if (log_callback) {
            QString log_str = QString("x: %1, z: %2").arg(velocity).arg(angular);
            log_callback(log_str);
        }
    }


    // [Topic] 오돔 콜백 (쿼터니언 -> 오일러 변환 포함)
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // 쿼터니언 -> 오일러 (Yaw) 계산
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        double theta = std::atan2(siny_cosp, cosy_cosp);

        if (pose_callback) pose_callback(x, y, theta);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!is_safety_on) return;

        float min_dist = 999.0;
        for (size_t i = 0; i < 30; ++i)
            if (msg->ranges[i] > 0.01 && msg->ranges[i] < min_dist) min_dist = msg->ranges[i];
        for (size_t i = msg->ranges.size() - 30; i < msg->ranges.size(); ++i)
            if (msg->ranges[i] > 0.01 && msg->ranges[i] < min_dist) min_dist = msg->ranges[i];

        if (min_dist < 0.5) {
            if (velocity > 0.0) {
                velocity = 0.0;
                if (log_callback) log_callback("");
            }
            if (warning_callback) warning_callback(true);
        } else {
            if (warning_callback) warning_callback(false);
        }
    }

    // [Service] 안전 모드 설정 콜백
    void safety_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        is_safety_on = request->data;
        response->success = true;
        response->message = is_safety_on ? "Safety ON" : "Safety OFF";
        if(log_callback) log_callback(is_safety_on ? "서비스 요청: 안전모드 켜짐" : "서비스 요청: 안전모드 꺼짐");
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_safety_;
    rclcpp_action::Client<Patrol>::SharedPtr action_client_;
};

#endif // MOVE_TURTLE_HPP_
