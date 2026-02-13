# 🐢 TurtleBot3 Advanced Control GUI

ROS 2 (C++)와 Qt5를 기반으로 제작된 터틀봇 통합 제어 및 모니터링 시스템입니다.

## 🚀 주요 기능 (기능 명세서)
| 기능 | 통신 방식 | 설명 |
|---|---|---|
| **수동 이동 제어** | `Topic (Pub)` | W/A/S/D 버튼을 통해 `/cmd_vel` 토픽을 발행하여 로봇 조종 |
| **로그 모니터링** | `내부 Signal` | 발행된 속도(Linear, Angular) 값을 GUI 리스트 뷰에 실시간 출력 |
| **위치 추적 (예정)** | `Topic (Sub)` | `/odom` 토픽을 구독하여 로봇의 현재 위치 표시 |
| **순찰 모드 (예정)** | `Action` | 사각형/삼각형 경로 자동 순찰 기능 지원 |
| **비상 정지 (예정)** | `Service` | 위험 상황 시 로봇의 구동을 즉각 차단 |

## 🛠️ 개발 환경
- Ubuntu 22.04 / ROS 2 Humble
- C++17 / Qt5 (Qt Designer)
- CMake (ament_cmake)

## 💻 실행 방법
```bash
cd ~/robot_ws
colcon build --packages-select my_turtle_gui
source install/setup.bash
ros2 run my_turtle_gui my_gui_node
