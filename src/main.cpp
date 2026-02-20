#include <QApplication>
#include "my_turtle_gui/mainwindow.hpp"
#include "my_turtle_gui/patrol_server.hpp"

/**
 * Main 함수: 프로그램의 전체 생명 주기를 관리
 */
int main(int argc, char *argv[])
{
    // 1. ROS 2 환경 초기화: 터미널 명령행 인자를 기반으로 통신 인프라 구축
    rclcpp::init(argc, argv);

    // 2. 노드 생성: GUI가 사용할 통신 노드와 실제 로직을 처리할 Patrol 서버 노드 생성
    // gui_node: 화면 갱신 및 명령 발행용 노드
    auto gui_node = std::make_shared<rclcpp::Node>("gui_node");

    // patrol_node: 액션 서버 및 PID 주행 로직을 가진 서버 노드
    auto patrol_node = std::make_shared<PatrolServer>();

    // 3. Executor 설정: 멀티스레드 실행기 생성
    // 두 개 이상의 노드가 각자 독립적인 콜백을 처리해야 하므로 MultiThreadedExecutor를 사용함
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(gui_node);
    executor.add_node(patrol_node);

    // 4. Background Thread: ROS 통신 전용 스레드 실행
    // Qt는 메인 스레드에서 화면을 그려야 함.
    // 따라서 ROS의 무한 루프(spin)를 별도의 스레드에서 돌려야 GUI가 멈추지 않음.
    std::thread ros_thread([&executor]() {
        executor.spin(); // ROS 통신 대기 루프 시작
    });

    // 5. Qt GUI 시작
    QApplication app(argc, argv); // Qt 애플리케이션 객체 생성
    MainWindow w(gui_node); // 메인 윈도우 생성 (GUI 노드 전달)
    w.show(); // 화면에 창을 띄움

    // app.exec()은 Qt의 이벤트 루프를 실행하며, 창을 닫을 때까지 여기서 대기함
    int result = app.exec();

    // 6. 종료 처리: 프로그램이 꺼질 때 안전하게 자원 해제
    executor.cancel();
    rclcpp::shutdown();

    // ROS 전용 스레드가 종료될 때까지 기다린 후 프로그램을 완전히 종료
    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return result;
}
