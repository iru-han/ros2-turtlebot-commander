#include "my_turtle_gui/mainwindow.hpp"
// ★ 빌드 시 .ui 파일이 아래 이름의 헤더 파일로 자동 변환됩니다!
#include "ui_main_window.h"

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), node_(node)
{
    ui->setupUi(this); // 디자인 덮어씌우기

    // 1. QoS 설정 (두 번째 예제의 Reliable 설정 적용)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // 2. Topic Pub
    pub_cmd_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);

    // 3. Topic Sub (odom-location)
    sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos_profile, std::bind(&MainWindow::odom_callback, this, std::placeholders::_1)
    );

    // 4. Topic Sub (scan-obstacle)
    auto qos = rclcpp::SensorDataQoS();
    sub_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&MainWindow::scan_callback, this, std::placeholders::_1)
    );

    // 5. Action Client (patrol)
    action_client_ = rclcpp_action::create_client<Patrol>(node_, "turtlebot3");

    // 생성자 내부
    safety_client_ = node_->create_client<std_srvs::srv::SetBool>("toggle_safety");

    // 8. 화면 안전 업데이트 신호 연결
    connect(this, &MainWindow::updateUiSignal, this, &MainWindow::updateUiSlot);

    // ★ 서비스 클라이언트 초기화
    safety_client_ = node_->create_client<std_srvs::srv::SetBool>("toggle_safety");
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_linear_vel_ = msg->twist.twist.linear.x;

    // GUI 스레드와 안전하게 통신하기 위해 시그널 발생
    QString status_log = QString("X: %1, Y: %2, Vel: %3")
    .arg(current_x_, 0, 'f', 2)
    .arg(current_y_, 0, 'f', 2)
    .arg(current_linear_vel_, 0, 'f', 2);

    emit updateUiSignal(current_x_, current_y_, false, status_log);
}

// ★ 5. 장애물 감지 함수 (파이썬 DetectObstacle 기능)
void MainWindow::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!is_safety_on_) return; // ★ 안전 모드 꺼져있으면 검사 안 함

    // 전방 60도 범위 (앞쪽 30도 + 뒤쪽 30도? 보통 인덱스로 처리)
    // 파이썬 코드: front_ranges = msg.ranges[0:30] + msg.ranges[-30:]
    float min_dist = 100.0;
    int scan_size = msg->ranges.size();

    for (int i=0; i<30; ++i) {
        float r = msg->ranges[i];
        if (r > 0.01 && r < min_dist) min_dist = r;
    }

    for (int i=scan_size-30; i<scan_size; ++i) {
        float r = msg->ranges[i];
        if (r > 0.01 && r < min_dist) min_dist = r;
    }

    if (min_dist < 0.3) {
        emit updateUiSignal(0, 0, true, "Obstacle!!");
        auto stop_msg = geometry_msgs::msg::Twist();
        pub_cmd_->publish(stop_msg);
    } else {
        emit updateUiSignal(0, 0, false, "");
    }
}

// UI 업데이트 함수들
void MainWindow::updateUiSlot(double x, double y, bool warning, QString log) {
    if (warning) {
        updateWarningUI(true);
        if (!log.isEmpty()) ui->listWidget->addItem(log);
    } else {
        if (ui->label_pos_x) ui->label_pos_x->setText(QString::number(x, 'f', 2));
        if (ui->label_pos_y) ui->label_pos_y->setText(QString::number(y, 'f', 2));
        updateWarningUI(false);
    }
}

void MainWindow::updateWarningUI(bool is_danger) {
    if (is_danger) {
        ui->label_warning->setText("충돌 위험!");
        ui->label_warning->setStyleSheet("QLabel { color : red; font-weight: bold; }");
    } else {
        ui->label_warning->setText("안전");
        ui->label_warning->setStyleSheet("QLabel { color : green; }");
    }
}

// 버튼 클릭 이벤트 함수 (예: ui->btn_safety)
void MainWindow::on_btn_safety_toggle_clicked() {
    if (!safety_client_->wait_for_service(std::chrono::seconds(1))) {
        ui->listWidget->addItem("Service Server not found!");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true; // 서버에서 토글하므로 값은 상관없음

    // 비동기 요청
    safety_client_->async_send_request(request, 
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    // 서버 응답 메시지에 따라 GUI의 안전 모드 변수도 업데이트
                    this->is_safety_on_ = response->message.find("ON") != std::string::npos;
                    
                    // UI 업데이트는 반드시 Signal을 통해서!
                    QString msg = QString::fromStdString(response->message);
                    emit updateUiSignal(0, 0, false, "GUI Safety: " + msg);
                }
            } catch (const std::exception & e) {
                emit updateUiSignal(0, 0, false, "Service Failed!");
            }
        });
}

// 버튼 클릭 동작 정의
void MainWindow::on_btn_go_clicked() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.2;
    pub_cmd_->publish(msg);
}
void MainWindow::on_btn_back_clicked() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = -0.2;
    pub_cmd_->publish(msg);
}
void MainWindow::on_btn_left_clicked() {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = 0.5;
    pub_cmd_->publish(msg);
}
void MainWindow::on_btn_right_clicked() {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = -0.5;
    pub_cmd_->publish(msg);
}
void MainWindow::on_btn_stop_clicked() {
    auto msg = geometry_msgs::msg::Twist();
    pub_cmd_->publish(msg);
}

void MainWindow::on_btn_patrol_square_clicked() {
    auto goal_msg = Patrol::Goal();
    goal_msg.goal.x = 1.0;
    goal_msg.goal.y = 1.0;
    goal_msg.goal.z = 1.0;

    auto opts = rclcpp_action::Client<Patrol>::SendGoalOptions();

    // ★ 결과 콜백 추가
    opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<Patrol>::WrappedResult & result) {
        QString final_msg;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                final_msg = "Patrol Success!";
                break;
            case rclcpp_action::ResultCode::ABORTED:
                final_msg = "Patrol Aborted (Obstacle?)";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                final_msg = "Patrol Canceled";
                break;
            default:
                final_msg = "Unknown Result";
                break;
        }
        // UI에 결과 출력
        emit updateUiSignal(0, 0, false, final_msg);
    };


    action_client_->async_send_goal(goal_msg, opts);
    ui->listWidget->addItem("Square Patrol sended!");
}

void MainWindow::on_btn_patrol_triangle_clicked() {
    auto goal_msg = Patrol::Goal();
    goal_msg.goal.x = 2.0;
    goal_msg.goal.y = 1.0;
    goal_msg.goal.z = 1.0;

    auto opts = rclcpp_action::Client<Patrol>::SendGoalOptions();

    // ★ 결과 콜백 추가
    opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<Patrol>::WrappedResult & result) {
        QString final_msg;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                final_msg = "Patrol Success!";
                break;
            case rclcpp_action::ResultCode::ABORTED:
                final_msg = "Patrol Aborted (Obstacle?)";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                final_msg = "Patrol Canceled";
                break;
            default:
                final_msg = "Unknown Result";
                break;
        }
        // UI에 결과 출력
        emit updateUiSignal(0, 0, false, final_msg);
    };

    action_client_->async_send_goal(goal_msg, opts);
    ui->listWidget->addItem("Triangle Patrol sended!");
}
