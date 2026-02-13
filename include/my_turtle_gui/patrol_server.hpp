#ifndef PATROL_SERVER_HPP_
#define PATROL_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlebot3_msgs/action/patrol.hpp"
#include <cmath>
#include <thread>
#include <algorithm> // clamp

class PatrolServer: public rclcpp::Node {
public:
  using Patrol = turtlebot3_msgs::action::Patrol;
  using GoalHandlePatrol = rclcpp_action::ServerGoalHandle<Patrol>;

  PatrolServer(); // 생성자

private:
  // 액션 서버 관련 콜백 함수들 선언
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Patrol::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePatrol> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandlePatrol> goal_handle);
  void execute(const std::shared_ptr<GoalHandlePatrol> goal_handle); // real_move

  // 센서 콜백
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ★ 수학 함수 (쿼터니언 -> 오일러)
  double get_yaw(const nav_msgs::msg::Odometry & odom);

  double calculate_distance(double x1, double y1, double x2, double y2);

  // ★ 이동 함수 (PID 적용, 파이썬의 go_front, turn 이식)
  bool move_straight(double target_distance, const std::shared_ptr<GoalHandlePatrol> goal_handle);
  bool rotate(double target_angle_deg, const std::shared_ptr<GoalHandlePatrol> goal_handle);

  // 변수들
  rclcpp_action::Server<Patrol>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  nav_msgs::msg::Odometry current_odom_;
  bool is_odom_received_ = false;
};

#endif
