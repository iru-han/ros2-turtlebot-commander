#include "my_turtle_gui/mainwindow.hpp"
#include "ui_main_window.h"

/**
 * GUI 창이 뜰 때 최초 1회 실행
 */
MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), node_(node)
{
    ui->setupUi(this);// Qt Designer로 만든 화면 레이아웃 로드

    // 1. 통신 품질(QoS) 설정: 신뢰성 있는 통신을 위해 최신 10개의 메시지를 보관
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // 2. Topic Publisher: 로봇에게 이동 명령(/cmd_vel)을 보내기 위한 통로 개설
    pub_cmd_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);

    // 3. Topic Subscriber: 로봇의 현재 위치(/odom) 정보를 실시간으로 받아옴
    sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos_profile, std::bind(&MainWindow::odom_callback, this, std::placeholders::_1)
    );

    // 4. Topic Subscriber: LiDAR 센서(/scan)를 통해 장애물 정보를 받아옴 (센서 데이터 전용 QoS 사용)
    auto qos = rclcpp::SensorDataQoS();
    sub_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&MainWindow::scan_callback, this, std::placeholders::_1)
    );

    // 5. Action Client: 사각형/삼각형 순찰이라는 작업을 요청하기 위한 액션 클라이언트 (긴 작업)
    action_client_ = rclcpp_action::create_client<Patrol>(node_, "turtlebot3");

    // 6. Service Client: 안전 모드를 켜고 끄는 작업을 요청하기 위한 서비스 클라이언트 (단발성)
    safety_client_ = node_->create_client<std_srvs::srv::SetBool>("toggle_safety");

    // 7. Qt Signal-Slot: ROS 통신 스레드와 GUI 화면 스레드를 안전하게 연결
    // ROS에서 데이터를 받으면(시그널), 화면 갱신 함수(슬롯)를 실행
    connect(this, &MainWindow::update_ui_signal, this, &MainWindow::update_ui_slot);
}

MainWindow::~MainWindow() {
    delete ui;
}

/**
 * Odom 콜백: 로봇의 위치 정보가 업데이트될 때마다 실행됨
 */
void MainWindow::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 메시지에서 x, y 좌표와 현재 선속도를 추출하여 멤버 변수에 저장
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_linear_vel_ = msg->twist.twist.linear.x;

    // 화면에 표시할 로그 문자열 생성 (소수점 2자리까지)
    QString status_log = QString("X: %1, Y: %2, Vel: %3")
    .arg(current_x_, 0, 'f', 2)
    .arg(current_y_, 0, 'f', 2)
    .arg(current_linear_vel_, 0, 'f', 2);

    // GUI 스레드에 화면 갱신을 요청 (직접 UI를 건드리지 않고 시그널을 보냄)
    emit update_ui_signal(current_x_, current_y_, false, "");
}

/**
 * Scan 콜백: LiDAR 센서로 주변 장애물을 실시간 감시
 */
void MainWindow::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!is_safety_on_) return; // 안전 모드가 꺼져있으면 감시 중단

    float min_dist = 100.0;
    int scan_size = msg->ranges.size();

    // 전방 60도(앞쪽 30도 + 뒤쪽 인덱스 30도) 범위 내 가장 가까운 거리 계산
    for (int i=0; i<30; ++i) {
        float r = msg->ranges[i];
        if (r > 0.01 && r < min_dist) min_dist = r;
    }

    for (int i=scan_size-30; i<scan_size; ++i) {
        float r = msg->ranges[i];
        if (r > 0.01 && r < min_dist) min_dist = r;
    }

    // 단계별 안전 로직
    if (min_dist <= 0.2) {
        // 1. :위험 충돌 직전. 즉시 후진하여 사고 방지
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = -0.1;
        pub_cmd_->publish(twist_msg);
        emit update_ui_signal(current_x_, current_y_, true, "위험! 충돌 직전, 후진 중.");
    } else if (min_dist > 0.2 && min_dist <= 0.32) {
        // 2. 경고: 전진 중일 때만 강제 정지
        if (current_linear_vel_ > 0.01) {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.0; // 정지!
            pub_cmd_->publish(twist_msg);
            emit update_ui_signal(current_x_, current_y_, true, "경고! 장애물 발견으로 정지");
        } else {
            emit update_ui_signal(current_x_, current_y_, true, "주의! 장애물 인접");
        }
    } else {
        // 3. 안전
        emit update_ui_signal(current_x_, current_y_, false, "");
    }
}

/**
 * UI 갱신 슬롯: ROS의 모든 데이터는 이 함수를 거쳐 화면에 출력됨
 */
void MainWindow::update_ui_slot(double x, double y, bool warning, QString log) {
    // 1. 공통 처리 (좌표 업데이트)
    if (ui->label_pos_x) ui->label_pos_x->setText("X: "+QString::number(x, 'f', 2));
    if (ui->label_pos_y) ui->label_pos_y->setText("Y: "+QString::number(y, 'f', 2));

    // 2. 상태에 따른 UI 변경
    update_warning_ui(warning);

    // 3. 로그 출력 (내용이 있다면 무조건 출력)
    if (!log.isEmpty()) {
        ui->listWidget->addItem(log);
        if (ui->listWidget->count() > 100) delete ui->listWidget->takeItem(0);
        ui->listWidget->scrollToBottom();
    }
}

void MainWindow::update_warning_ui(bool is_danger) {
    if (is_danger) {
        ui->label_warning->setText("충돌 위험");
        ui->label_warning->setStyleSheet("QLabel { color : red; font-weight: bold; }");
    } else {
        ui->label_warning->setText("안전");
        ui->label_warning->setStyleSheet("QLabel { color : green; }");
    }
}

/**
 * Service: 안전 모드 토글 버튼 클릭 시
 */
void MainWindow::on_btn_safety_toggle_clicked() {
    // 서비스 서버가 켜져있는지 확인
    if (!safety_client_->wait_for_service(std::chrono::seconds(1))) {
        emit update_ui_signal(current_x_, current_y_, false, "서비스 서버 연결 실패");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true; // 서버 내부 로직에 의해 토글됨

    // 비동기로 서버에 요청 전송
    auto response_received_callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
            // 서버의 응답 문자열에 따라 GUI 내부 변수 동기화
            this->is_safety_on_ = (response->message.find("ON") != std::string::npos);
            QString msg = QString::fromStdString(response->message);
            emit update_ui_signal(current_x_, current_y_, false, msg);
        }
    };
    safety_client_->async_send_request(request, response_received_callback);
}

/**
 * Manual Control: 수동 조종 버튼들
 */
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

/**
 * Stop: 정지 및 액션 취소
 */
void MainWindow::on_btn_stop_clicked() {
    bool is_patrol = false;

    // 1. 현재 실행 중인 순찰 액션이 있다면 취소 요청
    if (this->patrol_goal_handle_) {
        this->action_client_->async_cancel_goal(this->patrol_goal_handle_);
        this->patrol_goal_handle_ = nullptr; // 핸들 초기화

        is_patrol = true;
    }

    // 2. 물리적인 로봇 속도 0으로 초기화
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    pub_cmd_->publish(msg);

    if (is_patrol) {
        emit update_ui_signal(current_x_, current_y_, false, "순찰 중지");
    }

    RCLCPP_INFO(node_->get_logger(), "Stop Button Clicked");
}

void MainWindow::on_btn_patrol_square_clicked() { send_patrol_goal(1.0); }
void MainWindow::on_btn_patrol_triangle_clicked() { send_patrol_goal(2.0); }

/**
 * Action: 통합 순찰 명령 전송 함수
 */
void MainWindow::send_patrol_goal(double mode) {
    auto goal_msg = Patrol::Goal();
    goal_msg.goal.x = mode;
    goal_msg.goal.y = 1.0;
    goal_msg.goal.z = 1.0;

    auto opts = rclcpp_action::Client<Patrol>::SendGoalOptions();

    // Feedback: 순찰 중 서버가 보내는 중간 보고(오차 등)를 처리
    opts.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<Patrol>::SharedPtr handle) {
        if (!handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        } else {
            this->patrol_goal_handle_ = handle; // 여기서 저장해야 나중에 Stop 버튼이 취소할 수 있음!
        }
    };

    // Result: 순찰이 완전히 끝났을 때(성공/실패/취소) 처리
    opts.feedback_callback = [this](
        rclcpp_action::ClientGoalHandle<Patrol>::SharedPtr,
        const std::shared_ptr<const Patrol::Feedback> feedback)
    {
        QString log = QString::fromStdString(feedback->state);
        emit update_ui_signal(current_x_, current_y_, false, log);
    };

    action_client_->async_send_goal(goal_msg, opts);
    emit update_ui_signal(current_x_, current_y_, false, mode == 1.0 ? "사각형 순찰 시작" : "삼각형 순찰 시작");
    // ui->listWidget->addItem(mode == 1.0 ? "사각형 순찰 시작" : "삼각형 순찰 시작");
}
