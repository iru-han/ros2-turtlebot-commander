# [기능 명세서] 터틀봇3 GUI 제어 및 순찰 시스템

| 요구사항 명 | 주 기능 | 우선순위 | 필요 기술 | FE / BE | 인터페이스(API) 명세 | 비고 |
|------------|------------|--------|----------|-----------|---------|----------------------|------|
| 수동 주행 제어 | GUI 방향 버튼을 통한 로봇 실시간 원격 제어 (전/후/좌/우/정지) | 상 | Qt5, rclcpp | GUI | PUB /cmd_vel Type: geometry_msgs/Twist | 정지 시 모든 액션 취소 로직 포함 |
| 위치 모니터링 | 오도메트리 데이터를 수신하여 로봇의 현재 좌표(X, Y) 실시간 표시 | 상 | Odometry, Qt5 | GUI | SUB /odom Type: nav_msgs/Odometry | 쿼터니언 → 오일러(Yaw) 변환 적용 |
| 장애물 감지 및 대응 | LiDAR 데이터를 분석하여 전방 장애물 거리 측정 및 위험 시 자동 후진/정지 | 상 | LaserScan, rclcpp | Server | SUB /scan Type: sensor_msgs/LaserScan | 감지 범위: 전방 60도, 임계치: 0.2m ~ 0.32m |
| 안전 모드 토글 | 서비스 요청을 통해 서버의 충돌 방지 로직을 실시간으로 활성화/비활성화 | 중 | rclcpp (Service) | GUI / Server | SRV /toggle_safety Req: SetBool, Res: success/message | ReentrantCallbackGroup 적용 (데드락 방지) |
| 사각형 순찰 주행 | 액션 명령을 통해 1m x 1m 사각형 경로를 PID 제어로 자율 주행 | 상 | rclcpp_action, PID | GUI / Server | ACT /turtlebot3 Goal: mode=1.0, Feedback: state | 오돔 피드백 기반 위치 보정 수행 |
| 삼각형 순찰 주행 | 액션 명령을 통해 한 변이 1m인 정삼각형 경로를 PID 제어로 자율 주행 | 상 | rclcpp_action, PID | GUI / Server | ACT /turtlebot3 Goal: mode=2.0, Feedback: state | 120도 정밀 회전 및 각도 정규화 적용 |
| 지능형 로그 시스템 | 시스템 상태, 서비스 응답, 액션 피드백을 한글로 가독성 있게 출력 | 중 | Qt5 (QListWidget) | GUI | Signal-Slot update_ui_signal | 최신 로그 자동 스크롤 및 100개 제한 관리 |

## 1. 프로젝트 개요

본 프로젝트는 ROS 2 Humble 환경에서 Qt5 기반의 GUI를 통해 터틀봇3(TurtleBot3)를 정밀 제어하는 소프트웨어입니다. 단순한 원격 조종을 넘어, 센서 데이터 기반의 자율 주행(Action), 동적 상태 변경(Service), 그리고 실시간 상태 모니터링(Topic) 기능을 통합 구현하였습니다.

---

## 2. 시스템 아키텍처 및 통신 구조

본 시스템은 두 개의 주요 노드로 구성되어 멀티스레딩 방식으로 동작합니다.

- **GUI Node (Client)**: 사용자와의 인터페이스를 담당하며 명령 발행 및 상태 시각화 수행.
- **Patrol Server Node (Server)**: 실제 로봇 주행 알고리즘(PID 제어) 및 센서 데이터 분석 수행.

### 통신 방식:

- **Topic**: 연속적인 데이터 스트림 (위치, 센서값)
- **Service**: 일회성 명령 및 설정 변경 (안전 모드 토글)
- **Action**: 긴 시간이 소요되는 목표 지향적 작업 (순찰 주행)

---

## 3. 상세 기능 명세 (Detailed Specifications)

### 3.1 Topic: 실시간 데이터 발행 및 구독

| 토픽명 | 메시지 타입 | 방향 | 기능 설명 |
|--------|--------------|------|------------|
| /cmd_vel | geometry_msgs/msg/Twist | Pub | 수동 조작 및 순찰 알고리즘에 따른 속도 명령(선속도, 각속도) 발행 |
| /odom | nav_msgs/msg/Odometry | Sub | 로봇의 현재 위치 및 자세(Quaternion) 수신. 고정밀 주행의 피드백 소스로 활용 |
| /scan | sensor_msgs/msg/LaserScan | Sub | LiDAR 데이터를 수신하여 전방 60도 범위 내 장애물 유무 판별 |

🛠 기술적 특징:

- **QoS (Quality of Service) 최적화**: 레이저 데이터는 SensorDataQoS를 적용하여 저지연성을 확보하고, 오돔 데이터는 Reliable 설정을 통해 데이터 무결성을 보장함.
- **Quaternion to Euler Conversion**: 수신된 쿼터니언 데이터를 아래 수식을 통해 Yaw(각도) 값으로 변환하여 GUI에 표시 및 PID 제어에 활용.

$$
Yaw = \operatorname{atan2}(2(qw \cdot qz + qx \cdot qy), 1 - 2(qy^2 + qz^2))
$$

---

### 3.2 Service: 동적 안전 제어 시스템

- **서비스명**: /toggle_safety
- **인터페이스**: std_srvs/srv/SetBool
- **기능**: 로봇의 Safety Mode를 실시간으로 ON/OFF 함.

🛠 동작 로직:

- **Request**: GUI 버튼 클릭 시 서버로 현재 상태와 반대되는 요청 전송.
- **Server-side Toggle**: 서버 노드 내의 전역 상태 변수를 변경.

#### Collision Avoidance:

- **Mode ON**: 장애물 거리 0.3m 이내 감지 시 모든 주행 명령을 차단하고 강제 정지 명령 발행.
- **Mode OFF**: 사용자의 명령에 따른 강제 주행 허용.

- **Deadlock 방지**: ReentrantCallbackGroup을 사용하여 서비스 응답 중에도 다른 콜백이 실행될 수 있도록 설계.

---

### 3.3 Action: PID 기반 지능형 순찰 시스템

- **액션명**: turtlebot3
- **인터페이스**: turtlebot3_msgs/action/Patrol
- **기능**: 사각형 또는 삼각형 경로를 자율 주행함.

🛠 정밀 제어 알고리즘 (PID Control):

단순 타이머 기반 주행이 아닌, 오돔 피드백을 이용한 PID 제어를 구현하여 정밀도를 높임.

- **Proportional (P)**: 목표치와의 오차에 비례하여 속도 제어.
- **Integral (I)**: 바닥 마찰 등으로 인한 누적 오차를 제거하여 목표 지점 정밀 도달.
- **Derivative (D)**: 목표 지점 도달 시 오버슈트(Overshoot)를 방지하기 위한 감속 제어.

🛠 액션 생명 주기:

- **Goal**: 사각형(Mode 1) 또는 삼각형(Mode 2) 목표 전달.

- **Feedback**:
  - 실시간 거리 오차(Error) 데이터 전송.
  - 장애물 감지 시 주행을 멈추고 "일시 정지됨: 장애물 감지" 상태를 GUI로 전송.

- **Result**:
  - 모든 경로 완주 시 SUCCEEDED 반환.
  - 장애물로 인해 주행 불가능 시 ABORTED 반환.

---

## 4. GUI 설계 및 인터페이스 (HMI)

### Control Section:
- 5축 버튼(전/후/좌/우/정지)을 통한 직관적 원격 제어.

### Telemetry Section:
- 실시간 좌표 (x, y, theta) 시각화.
- QLabel의 Style Sheet를 동적으로 변경하여 안전(Green)/위험(Red) 상태를 시각적 강조.

### Log Console (QListWidget):
- 모든 통신 결과(Service 응답, Action 피드백)를 시간순으로 기록.
- scrollToBottom() 기능을 통해 최신 로그 자동 추적.

---

## 5. 프로젝트 파일 구조

```plaintext
my_turtle_gui/
├── CMakeLists.txt          # 패키지 빌드 설정 (Qt5, ROS2 의존성 관리)
├── package.xml             # 의존성 패키지 명시 (sensor_msgs, std_srvs 등)
├── include/
│   └── my_turtle_gui/      # 헤더 파일 (클래스 선언부)
├── src/
│   ├── main.cpp            # 노드 실행 및 멀티스레드 익스큐터 설정
│   ├── mainwindow.cpp      # GUI 로직 및 ROS Client 구현
│   └── patrol_server.cpp   # PID 주행 및 ROS Server 구현
└── ui/
    └── main_window.ui      # Qt Designer로 설계된 UI 레이아웃
```

## 6. 실행 방법
# 워크스페이스 이동
cd ~/robot_ws

# 패키지 빌드 (my_turtle_gui 패키지만 선택 빌드)
colcon build --packages-select my_turtle_gui

# 환경 설정 반영 (빌드 후 필수)
source install/setup.bash

# 터틀봇 모델 설정 (본인 모델에 맞게 선택: burger 또는 waffle_pi)
export TURTLEBOT3_MODEL=burger

# 시뮬레이션 실행 (새 터미널)
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py

# GUI 및 패트롤 서버 통합 노드 실행
ros2 run my_turtle_gui my_gui_node
