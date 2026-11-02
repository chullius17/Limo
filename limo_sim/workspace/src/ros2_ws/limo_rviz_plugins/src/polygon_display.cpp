// Copyright 2026 Giulio Cataldo
// SPDX-License-Identifier: Apache-2.0

#include <QMetaObject>
#include <QThread>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_default_plugins/displays/polygon/polygon_display.hpp"

namespace limo_rviz_plugins
{

// Foxy's MessageFilterDisplay calls processMessage from the TF listener thread
// when a missing transform arrives. PolygonDisplay then changes OGRE geometry
// without that thread owning an OpenGL context, racing the main render thread.
class PolygonDisplay : public rviz_default_plugins::displays::PolygonDisplay
{
protected:
  void processMessage(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) override
  {
    if (QThread::currentThread() == thread()) {
      rviz_default_plugins::displays::PolygonDisplay::processMessage(msg);
      return;
    }
    // Passing this as the context also discards the queued callback if the
    // display is deleted before the GUI event loop handles it.
    QMetaObject::invokeMethod(this, [this, msg]() {
      if (isEnabled()) {
        rviz_default_plugins::displays::PolygonDisplay::processMessage(msg);
      }
    }, Qt::QueuedConnection);
  }
};

}  // namespace limo_rviz_plugins

PLUGINLIB_EXPORT_CLASS(limo_rviz_plugins::PolygonDisplay, rviz_common::Display)
