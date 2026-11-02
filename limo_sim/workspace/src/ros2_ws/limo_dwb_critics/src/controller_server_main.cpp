// Copyright 2026 Giulio Cataldo
// SPDX-License-Identifier: Apache-2.0

// Foxy must deactivate/clean up the controller while its ROS context is valid.
// Its default SIGINT handler shuts down that context before lifecycle cleanup.
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <stdexcept>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_controller/nav2_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::uninstall_signal_handlers();
  std::signal(SIGINT, request_stop);
  // Keep SIGTERM's default disposition as launch's forced-stop fallback.
  int result = EXIT_SUCCESS;
  std::shared_ptr<nav2_controller::ControllerServer> node;
  try {
    node = std::make_shared<nav2_controller::ControllerServer>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    while (!stop_requested && rclcpp::ok()) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    executor.remove_node(node->get_node_base_interface());
    if (rclcpp::ok()) {
      using State = lifecycle_msgs::msg::State;
      RCLCPP_INFO(node->get_logger(), "Stopping controller before ROS shutdown");
      if (node->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
        if (node->deactivate().id() != State::PRIMARY_STATE_INACTIVE) {
          throw std::runtime_error("Controller deactivation failed");
        }
      }
      if (node->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
        if (node->cleanup().id() != State::PRIMARY_STATE_UNCONFIGURED) {
          throw std::runtime_error("Controller cleanup failed");
        }
      }
    }
  } catch (const std::exception & error) {
    std::fprintf(stderr, "Controller server failed: %s\n", error.what());
    result = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  node.reset();
  // Same opt-in workaround as CV: all lifecycle/ROS cleanup has finished.
  const char * fast_exit = std::getenv("LIMO_FAST_EXIT");
  if (fast_exit && std::strcmp(fast_exit, "1") == 0) {
    std::fflush(nullptr);
    std::_Exit(result);
  }
  return result;
}
