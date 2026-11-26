#include "limo_map/map_save_window.hpp"

#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QTime>
#include <QVBoxLayout>
#include <utility>

namespace limo_map
{
MapSaveWindow::MapSaveWindow(rclcpp::Node::SharedPtr node, QWidget * parent)
: QWidget(parent), node_(std::move(node))
{
  setWindowTitle("LIMO Map Saver");
  setWindowFlag(Qt::WindowStaysOnTopHint, true);
  setMinimumWidth(360);
  auto * layout = new QVBoxLayout(this);
  status_label_ = new QLabel("Press the button to save the current map.", this);
  status_label_->setObjectName("status_label");
  status_label_->setTextFormat(Qt::PlainText);
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  save_button_ = new QPushButton("SAVE MAP", this);
  save_button_->setObjectName("save_button");
  save_button_->setStyleSheet(
    "background-color: #0056b3; color: white; font-weight: bold; "
    "font-size: 16px; padding: 12px; border-radius: 5px;");
  connect(save_button_, &QPushButton::clicked, this, [this] {request_save();});
  layout->addWidget(save_button_);

  auto * terminal_label = new QLabel("MAP SAVER TERMINAL", this);
  terminal_label->setStyleSheet("font-weight: bold;");
  layout->addWidget(terminal_label);
  terminal_ = new QTextEdit(this);
  terminal_->setObjectName("terminal");
  terminal_->setReadOnly(true);
  terminal_->setMinimumHeight(180);
  terminal_->setStyleSheet(
    "background-color: #101418; color: #d7e0e7; font-family: monospace; "
    "font-size: 12px; padding: 6px;");
  layout->addWidget(terminal_);

  const auto service = node_->declare_parameter<std::string>(
    "save_service", "/limo/map_package/offline/map_saver/save_map");
  const auto topic = node_->declare_parameter<std::string>(
    "status_topic", "/limo/map_package/offline/map_saver/status");
  client_ = node_->create_client<std_srvs::srv::Trigger>(service);
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic, rclcpp::QoS(10).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      append_status(QString::fromStdString(msg->data));
    });
  RCLCPP_INFO(node_->get_logger(), "Map save GUI targeting %s", service.c_str());
}

void MapSaveWindow::request_save()
{
  if (!save_button_->isEnabled()) {return;}
  save_button_->setEnabled(false);
  status_label_->setStyleSheet("");
  status_label_->setText("Saving map...");
  try {
    if (!client_->service_is_ready()) {
      show_result(false, QString("Service '%1' is not available").arg(client_->get_service_name()));
      return;
    }
    client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          show_result(response->success, QString::fromStdString(response->message));
        } catch (const std::exception & error) {
          show_result(false, QString("Map save request failed: %1").arg(error.what()));
        }
      });
  } catch (const std::exception & error) {
    show_result(false, QString("Map save request failed: %1").arg(error.what()));
  }
}

void MapSaveWindow::show_result(bool success, const QString & message)
{
  const auto color = success ? "#2e7d32" : "#c62828";
  status_label_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(color));
  status_label_->setText(message);
  save_button_->setEnabled(true);
  const auto text = message.toStdString();
  if (success) {RCLCPP_INFO(node_->get_logger(), "%s", text.c_str());}
  else {RCLCPP_ERROR(node_->get_logger(), "%s", text.c_str());}
}

void MapSaveWindow::append_status(const QString & message)
{
  // Insert plain text: paths and backend messages may contain '<' or '&'.
  terminal_->append(QString());
  terminal_->insertPlainText(QTime::currentTime().toString("HH:mm:ss") + "  " + message);
}
}  // namespace limo_map
