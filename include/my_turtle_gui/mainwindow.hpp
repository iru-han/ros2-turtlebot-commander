#ifndef MAINWINDOW_HPP_
#define MAINWINDOW_HPP_

#include <QMainWindow>
#include <QCloseEvent>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlebot3_msgs/action/patrol.hpp"
#include "std_srvs/srv/set_bool.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// =====================================
// 메인 윈도우
// =====================================
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void updateUiSignal(double x, double y, bool warning, QString log);

private slots:
    // 버튼 클릭 슬롯들
    void on_btn_go_clicked();
    void on_btn_back_clicked();
    void on_btn_left_clicked();
    void on_btn_right_clicked();
    void on_btn_stop_clicked();
    void on_btn_patrol_square_clicked();
    void on_btn_patrol_triangle_clicked();
    void on_btn_safety_clicked();

    void updateUiSlot(double x, double y, bool warning, QString log);

    void updateWarningUI(bool is_danger);

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node_;

    // Publisher: 로봇에게 명령 전달
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

    // Subscription: 로봇의 상태(오돔, 스캔) 수신
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    // 현재 로봇 상태 저장 변수
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_linear_vel_ = 0.0;

    // action client
    using Patrol = turtlebot3_msgs::action::Patrol;
    rclcpp_action::Client<Patrol>::SharedPtr action_client_;

    // ★ 추가: 데이터를 받았을 때 실행할 콜백 함수들
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif // MAINWINDOW_HPP_
