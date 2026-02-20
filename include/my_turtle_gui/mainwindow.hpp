#ifndef MAINWINDOW_HPP_
#define MAINWINDOW_HPP_

// Qt 관련 헤더
#include <QMainWindow>
#include <QCloseEvent>

// ROS 2 핵심 및 액션 헤더
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// ROS 2 메시지 타입 헤더
#include "geometry_msgs/msg/twist.hpp"      // 로봇의 속도(선속도, 각속도) 명령용
#include "nav_msgs/msg/odometry.hpp"        // 로봇의 위치 및 자세 데이터용
#include "sensor_msgs/msg/laser_scan.hpp"   // LiDAR 센서 데이터(장애물 감지)용
#include "turtlebot3_msgs/action/patrol.hpp" // 순찰 액션(사각형/삼각형 정찰) 정의용
#include "std_srvs/srv/set_bool.hpp"        // 안전 모드 ON/OFF 서비스 통신용

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// 코드 가독성을 위해 길고 복잡한 타입 이름에 별칭을 부여
using Patrol = turtlebot3_msgs::action::Patrol;

/**
 * @class MainWindow
 * @brief 터틀봇 제어 GUI의 메인 클래스
 * QMainWindow를 상속받아 화면을 구성하고, 내부에 ROS 2 통신 기능을 포함
 */
class MainWindow : public QMainWindow {
    Q_OBJECT // Qt의 시그널-슬롯 기능을 사용하기 위한 필수 매크로

public:
    /**
     * @brief ROS 통신 스레드에서 받은 데이터를 안전하게 GUI 스레드로 넘겨주는 역할
     * ROS 콜백 함수는 배경 스레드에서 돌기 때문에 직접 UI를 건드리면 크래시가 날 수 있어 이 시그널을 사용
     */
    explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void update_ui_signal(double x, double y, bool warning, QString log);

private slots:
    // 버튼 클릭 슬롯들
    void on_btn_go_clicked();               // 로봇 전진
    void on_btn_back_clicked();             // 로봇 후진
    void on_btn_left_clicked();             // 로봇 좌회전
    void on_btn_right_clicked();            // 로봇 우회전
    void on_btn_stop_clicked();             // 정지 및 현재 액션 취소
    void on_btn_patrol_square_clicked();    // 사각형 정찰 시작
    void on_btn_patrol_triangle_clicked();  // 삼각형 정찰 시작
    void on_btn_safety_toggle_clicked();    // [Service] 안전 모드 ON/OFF 전환

    // 시그널을 받아 실제 화면(라벨, 리스트 위젯 등)을 갱신하는 함수
    void update_ui_slot(double x, double y, bool warning, QString log);

    // 위험 상태에 따라 UI 색상이나 문구를 변경하는 시각적 효과 담당
    void update_warning_ui(bool is_danger);

private:
    Ui::MainWindow *ui; // Qt Designer로 설계한 UI 객체 포인터
    rclcpp::Node::SharedPtr node_; // ROS 2 노드 핸들 (통신의 중심)

// 1. Publisher: 로봇에게 실제 이동 명령(/cmd_vel)을 발행함
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

    // 2. Subscription: 로봇의 상태 정보를 실시간으로 받아옴(구독)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;   // 위치 정보
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_; // 센서 정보

    // 3. Service Client: "안전 모드 바꿔줘!"라고 서버에 요청을 보냄
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr safety_client_;

    // 4. Action Client: "순찰해!"라고 명령하고 피드백을 실시간으로 수신함
    rclcpp_action::Client<Patrol>::SharedPtr action_client_;

    // 5. Goal Handle: 현재 진행 중인 액션 명령을 추적하고 취소할 때 사용함
    rclcpp_action::ClientGoalHandle<Patrol>::SharedPtr patrol_goal_handle_;

    // 현재 로봇 상태 저장 변수
    double current_x_ = 0.0;           // 로봇의 현재 X 좌표
    double current_y_ = 0.0;           // 로봇의 현재 Y 좌표
    double current_linear_vel_ = 0.0;  // 로봇의 현재 속도
    bool is_safety_on_ = true;         // 현재 안전 모드가 활성화되어 있는지 여부

    // action client
    rclcpp_action::Client<Patrol>::SharedPtr action_client_;

    // 데이터가 도착했을 때 자동으로 호출되는 콜백 함수들
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // 사각형/삼각형 순찰 명령 전송 시 중복되는 코드를 처리하는 통합 함수
    void send_patrol_goal(double mode);
};

#endif // MAINWINDOW_HPP_
