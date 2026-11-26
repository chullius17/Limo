#include "limo_map/map_save_window.hpp"

#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
void check(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
void wait_for(
  rclcpp::Executor & executor, const std::function<bool()> & condition,
  std::chrono::milliseconds timeout = std::chrono::seconds(6))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!condition() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(5));
    QApplication::processEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  check(condition(), "Timed out waiting for GUI/ROS interaction");
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  try {
    const std::string service = "/limo_map_gui_test/save", topic = "/limo_map_gui_test/status";
    auto node = std::make_shared<rclcpp::Node>("limo_map_save_gui", rclcpp::NodeOptions()
        .parameter_overrides({rclcpp::Parameter("save_service", service),
          rclcpp::Parameter("status_topic", topic)}));
    auto backend = std::make_shared<rclcpp::Node>("limo_map_gui_backend_test");
    auto status_pub = backend->create_publisher<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).reliable().transient_local());
    std_msgs::msg::String status;
    status.data = "Mapper ready <robot & camera>";
    status_pub->publish(status);
    limo_map::MapSaveWindow window(node);
    auto * button = window.findChild<QPushButton *>("save_button");
    auto * label = window.findChild<QLabel *>("status_label");
    auto * terminal = window.findChild<QTextEdit *>("terminal");
    check(button && label && terminal, "Missing save controls");
    window.show();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(backend);
    // No mapper: report the error immediately and leave the button usable.
    button->click();
    check(button->isEnabled() && label->text().contains("not available"), "Unavailable service not reported");
    wait_for(executor, [&] {return terminal->toPlainText().contains(QString::fromStdString(status.data));});

    unsigned requests = 0;
    bool success = true;
    auto server = backend->create_service<std_srvs::srv::Trigger>(service,
      [&](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        ++requests;
        response->success = success;
        response->message = success ? "Saved <complete & laser>" : "Cannot save: no reference map";
      });
    auto observer = node->create_client<std_srvs::srv::Trigger>(service);
    wait_for(executor, [&] {return observer->service_is_ready();});
    button->click();
    check(!button->isEnabled() && label->text() == "Saving map...", "Save request must be asynchronous");
    button->click();  // A disabled button must not send another request.
    wait_for(executor, [&] {return button->isEnabled();});
    check(requests == 1 && label->text() == "Saved <complete & laser>", "Successful save not shown");
    check(label->styleSheet().contains("#2e7d32"), "Successful save must use success feedback");
    success = false;
    button->click();
    wait_for(executor, [&] {return button->isEnabled();});
    check(requests == 2 && label->text().contains("no reference map"), "Backend save failure not shown");
    check(label->styleSheet().contains("#c62828"), "Save failure must use error feedback");

    server.reset();
    wait_for(executor, [&] {return !observer->service_is_ready();});
    button->click();
    check(button->isEnabled() && label->text().contains("not available"), "Mapper disconnect not handled");
    window.close();
    executor.remove_node(node);
    executor.remove_node(backend);
    std::cout << "Qt C++ GUI: async save, duplicate clicks, errors, disconnect and latched status passed\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
