#include <QApplication>
#include "my_turtle_gui/mainwindow.hpp"
#include "my_turtle_gui/patrol_server.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 1. GUI Node
    auto gui_node = std::make_shared<rclcpp::Node>("gui_node");

    // 2. Patrol Server Node
    auto patrol_node = std::make_shared<PatrolServer>();

    // 3. Executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(gui_node);
    executor.add_node(patrol_node);

    // 4. Thread
    std::thread ros_thread([&executor]() {
        executor.spin();
    });

    // 5. Qt GUI
    QApplication app(argc, argv);
    MainWindow w(gui_node);
    w.show();

    int result = app.exec();

    // Close
    executor.cancel();
    rclcpp::shutdown();
    ros_thread.join();

    return result;
}
