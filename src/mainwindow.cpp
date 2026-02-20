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
    connect(this, &MainWindow::update_ui_signal, this, &MainWindow::update_ui_slot);

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

    emit update_ui_signal(current_x_, current_y_, false, status_log);
}

// ★ 5. 장애물 감지 함수 (파이썬 DetectObstacle 기능)
void MainWindow::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!is_safety_on_) return; // ★ 안전 모드 꺼져있으면 검사 안 함

    // 전방 60도 범위 (앞쪽 30도 + 뒤쪽 30도? 보통 인덱스로 처리)
    // 파이썬 코드: front_ranges = msg.ranges[0:30] + msg.ranges[-30:]
    float min_dist = 100.0;
    int scan_size = msg->ranges.size();

    // 전방 범위 스캔
    for (int i=0; i<30; ++i) {
        float r = msg->ranges[i];
        if (r > 0.01 && r < min_dist) min_dist = r;
    }

    for (int i=scan_size-30; i<scan_size; ++i) {
        float r = msg->ranges[i];
        if (r > 0.01 && r < min_dist) min_dist = r;
    }

    if (min_dist <= 0.2) {
        // [1단계] 너무 가까움 -> 무조건 후진
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = -0.1;
        pub_cmd_->publish(twist_msg);
        emit update_ui_signal(current_x_, current_y_, true, "위험! 후진 중.");
    }
    else if (min_dist > 0.2 && min_dist <= 0.32) {
        // [2단계] 경고 구간 -> "전진 중일 때만" 멈추게 함
        // current_linear_vel_이 0보다 크다는 건 앞으로 가려 한다는 뜻입니다.
        if (current_linear_vel_ > 0.01) {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.0; // 정지!
            pub_cmd_->publish(twist_msg);
            emit update_ui_signal(current_x_, current_y_, true, "경고! 장애물 앞 정지");
        } else {
            // 앞으로 가는 중이 아니라면(회전 등) 경고만 띄우고 명령은 방해하지 않음
            emit update_ui_signal(current_x_, current_y_, true, "(주의) 장애물 근처");
        }
    }
    else {
        // [3단계] 안전
        emit update_ui_signal(current_x_, current_y_, false, "안전 거리 확보");
    }
}

// UI 업데이트 함수들
// mainwindow.cpp 파일 수정
void MainWindow::update_ui_slot(double x, double y, bool warning, QString log) {
    if (warning) {
        // 경고(위험) 상태일 때 로그 추가
        update_warning_ui(true);
        if (!log.isEmpty()) {
            ui->listWidget->addItem(log);
            if (ui->listWidget->count() > 100) delete ui->listWidget->takeItem(0);
            ui->listWidget->scrollToBottom();
        }
    } else {
        // 일반 상태(순찰 피드백, 시스템 메시지 등)
        if (ui->label_pos_x) ui->label_pos_x->setText(QString::number(x, 'f', 2));
        if (ui->label_pos_y) ui->label_pos_y->setText(QString::number(y, 'f', 2));
        update_warning_ui(false);
    }
}

void MainWindow::update_warning_ui(bool is_danger) {
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
        ui->listWidget->addItem("서비스 서버를 찾을 수 없습니다!");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true; // 서버에서 토글하므로 값은 상관없음

    // 비동기 요청 및 콜백 처리
    auto response_received_callback = [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
            // ★ 서버가 보낸 한글 메시지에 "ON"이 포함되어 있는지 확인하여 GUI 상태 동기화
            // std::string의 find를 사용합니다.
            this->is_safety_on_ = (response->message.find("ON") != std::string::npos);

            // ★ UI 시그널을 통해 한글 메시지를 listWidget에 출력합니다.
            QString msg = QString::fromStdString(response->message);
            ui->listWidget->addItem(msg);
        }
    };
    safety_client_->async_send_request(request, response_received_callback);
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
    bool is_patrol = false;

    // 1. 진행 중인 액션 목표 취소
    if (this->patrol_goal_handle_) {
        this->action_client_->async_cancel_goal(this->patrol_goal_handle_);
        this->patrol_goal_handle_ = nullptr; // 핸들 초기화

        is_patrol = true;
    }

    // 2. 로봇을 물리적으로 멈추기 위해 속도 0 전송
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    pub_cmd_->publish(msg);

    if (is_patrol) {
        ui->listWidget->addItem("순찰 중지");
        ui->listWidget->scrollToBottom();
    }

    RCLCPP_INFO(node_->get_logger(), "Stop Button Clicked");
}

void MainWindow::on_btn_patrol_square_clicked() { send_patrol_goal(1.0); }
void MainWindow::on_btn_patrol_triangle_clicked() { send_patrol_goal(2.0); }

// ★ 중복을 제거한 통합 액션 전송 함수
void MainWindow::send_patrol_goal(double mode) {
    auto goal_msg = Patrol::Goal();
    goal_msg.goal.x = mode;
    goal_msg.goal.y = 1.0;
    goal_msg.goal.z = 1.0;

    auto opts = rclcpp_action::Client<Patrol>::SendGoalOptions();

    // 1. 피드백 콜백: 서버가 보내는 "일시 정지" 또는 "주행 중" 메시지 표시
    // ★ 추가: 서버가 목표를 수락하면 이 핸들을 변수에 저장함
    opts.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<Patrol>::SharedPtr handle) {
        if (!handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        } else {
            this->patrol_goal_handle_ = handle; // 여기서 저장해야 나중에 Stop 버튼이 취소할 수 있음!
        }
    };

    // 2. 결과 콜백: 최종 성공/실패 메시지 표시
    opts.feedback_callback = [this](
        rclcpp_action::ClientGoalHandle<Patrol>::SharedPtr,
        const std::shared_ptr<const Patrol::Feedback> feedback)
    {
        QString log = QString::fromStdString(feedback->state);
        // [수정] 좌표 깜빡임 방지: 0 대신 현재 멤버 변수 값 전달
        emit update_ui_signal(current_x_, current_y_, false, log);
    };

    action_client_->async_send_goal(goal_msg, opts);
    ui->listWidget->addItem(mode == 1.0 ? "사각형 순찰 시작!" : "삼각형 순찰 시작!");
}
