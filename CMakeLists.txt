cmake_minimum_required(VERSION 3.8)
project(my_turtle_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(std_srvs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(turtlebot3_msgs REQUIRED) # patrol
find_package(std_srvs REQUIRED)

# Qt5 자동 변환 세팅
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ui) # ui folder

set(UI_FILES ui/main_window.ui)

# ★ 수정 포인트 1: 헤더 파일이 있는 include 폴더를 컴파일러가 찾을 수 있게 알려줌
include_directories(include)

# ★ 수정 포인트 2: 타겟 이름을 my_gui_node로 통일하고, mainwindow.cpp 추가!
add_executable(my_gui_node
  src/main.cpp
  src/mainwindow.cpp
  src/patrol_server.cpp
  include/my_turtle_gui/mainwindow.hpp
  include/my_turtle_gui/move_turtle.hpp
  ${UI_FILES}
)

# ★ 수정 포인트 3: 위에서 만든 my_gui_node 에 라이브러리 연결
target_link_libraries(my_gui_node Qt5::Widgets)
ament_target_dependencies(my_gui_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  nav_msgs
  turtlebot3_msgs
  std_srvs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 설치 경로 지정
install(TARGETS
  my_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
