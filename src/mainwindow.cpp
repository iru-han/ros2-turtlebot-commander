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


    // 6. 버튼 클릭 이벤트 연결
    connect(ui->btn_go, &QPushButton::clicked, this, &MainWindow::on_btn_go_clicked);
    connect(ui->btn_back, &QPushButton::clicked, this, &MainWindow::on_btn_back_clicked);
    connect(ui->btn_left, &QPushButton::clicked, this, &MainWindow::on_btn_left_clicked);
    connect(ui->btn_right, &QPushButton::clicked, this, &MainWindow::on_btn_right_clicked);
    connect(ui->btn_stop, &QPushButton::clicked, this, &MainWindow::on_btn_stop_clicked);

    // 7. ★ [Action] 순찰 버튼 연결
    if (ui->btn_patrol_square) connect(ui->btn_patrol_square, &QPushButton::clicked, this, &MainWindow::on_btn_patrol_square_clicked);
    if (ui->btn_patrol_triangle) connect(ui->btn_patrol_triangle, &QPushButton::clicked, this, &MainWindow::on_btn_patrol_triangle_clicked);

    // 8. 화면 안전 업데이트 신호 연결
    connect(this, &MainWindow::updateUiSignal, this, &MainWindow::updateUiSlot);

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
void MainWindow::on_btn_safety_clicked() {
    using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;

    // 1. 서비스 클라이언트 생성
    auto client = node_->create_client<std_srvs::srv::SetBool>("toggle_safety");

    // 2. 서버 연결 대기
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        ui->listWidget->addItem("Service not available");
        return;
    }

    // 3. 요청 데이터 설정 (현재 상태의 반대로 요청)
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    static bool current_state = true;
    current_state = !current_state;
    request->data = current_state;

    // 4. 비동기 요청 및 콜백 처리 (두 번째 코드의 lambda 방식)
    using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto response = future.get();
        if (response->success) {
            // UI에 서비스 결과 표시
            emit updateUiSignal(0, 0, false, QString::fromStdString(response->message));
        }
    };

    client->async_send_request(request, response_received_callback);
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
    action_client_->async_send_goal(goal_msg, opts);
    ui->listWidget->addItem("Square Patrol sended!");
}

void MainWindow::on_btn_patrol_triangle_clicked() {
    auto goal_msg = Patrol::Goal();
    goal_msg.goal.x = 2.0;
    goal_msg.goal.y = 1.0;
    goal_msg.goal.z = 1.0;

    auto opts = rclcpp_action::Client<Patrol>::SendGoalOptions();
    action_client_->async_send_goal(goal_msg, opts);
    ui->listWidget->addItem("Triangle Patrol sended!");
}
