#ifndef LIMO_MAP__MAP_SAVE_WINDOW_HPP_
#define LIMO_MAP__MAP_SAVE_WINDOW_HPP_

#include <QWidget>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class QLabel;
class QPushButton;
class QTextEdit;

namespace limo_map
{
// Spin this node on the Qt thread. All requests are asynchronous, so callbacks
// can update widgets directly without cross-thread access or blocking the UI.
class MapSaveWindow : public QWidget
{
public:
  explicit MapSaveWindow(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);

private:
  void request_save();
  void show_result(bool success, const QString & message);
  void append_status(const QString & message);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  QLabel * status_label_;
  QPushButton * save_button_;
  QTextEdit * terminal_;
};
}  // namespace limo_map
#endif  // LIMO_MAP__MAP_SAVE_WINDOW_HPP_
