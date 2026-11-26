#include "limo_map/map_save_window.hpp"

#include <QApplication>
#include <QTimer>
#include <chrono>
#include <csignal>
#include <string>
#include <vector>

namespace
{
volatile std::sig_atomic_t shutdown_requested = 0;
void request_shutdown(int) {shutdown_requested = 1;}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // Qt receives its own command-line options, without ROS remaps/parameters.
    auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<char *> qt_argv;
    for (auto & argument : arguments) {qt_argv.push_back(&argument[0]);}
    int qt_argc = static_cast<int>(qt_argv.size());
    qt_argv.push_back(nullptr);
    QApplication app(qt_argc, qt_argv.data());
    std::signal(SIGINT, request_shutdown);
    std::signal(SIGTERM, request_shutdown);

    auto node = std::make_shared<rclcpp::Node>("limo_map_save_gui");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    limo_map::MapSaveWindow window(node);
    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, &app, [&] {
      if (shutdown_requested || !rclcpp::ok()) {app.quit(); return;}
      try {
        executor.spin_some(std::chrono::milliseconds(5));
      } catch (const std::exception & error) {
        RCLCPP_ERROR(node->get_logger(), "GUI executor failed: %s", error.what());
        app.exit(1);
      }
    });
    // Both event loops run on the UI thread. Service calls never wait for a reply.
    ros_timer.start(20);
    window.show();
    window.raise();
    window.activateWindow();
    const int result = app.exec();
    ros_timer.stop();
    executor.remove_node(node);
    rclcpp::shutdown();
    return result;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("limo_map_save_gui"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
