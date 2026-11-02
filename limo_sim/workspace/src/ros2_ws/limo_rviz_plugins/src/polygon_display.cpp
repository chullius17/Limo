// Copyright 2026 Giulio Cataldo
// SPDX-License-Identifier: Apache-2.0

#include "limo_rviz_plugins/latest_message.hpp"
#include <OgreSceneNode.h>
#include "pluginlib/class_list_macros.hpp"
#include "rviz_default_plugins/displays/polygon/polygon_display.hpp"

namespace limo_rviz_plugins
{

// Foxy's MessageFilterDisplay calls processMessage from the TF listener thread
// when a missing transform arrives. PolygonDisplay then changes OGRE geometry
// without that thread owning an OpenGL context, racing the main render thread.
class PolygonDisplay : public rviz_default_plugins::displays::PolygonDisplay
{
public:
  void onInitialize() override
  {
    rviz_default_plugins::displays::PolygonDisplay::onInitialize();
    // Nav2 publishes footprint vertices at z=0, the same plane as the maps.
    // Lift the outlines slightly so costmap tiles cannot hide them.
    scene_node_->setPosition(0.0f, 0.0f, 0.03f);
  }

  ~PolygonDisplay() override
  {
    // Disconnect before destroying the slot accessed by TF callbacks.
    unsubscribe();
  }

  void update(float wall_dt, float ros_dt) override
  {
    auto message = pending_.take();
    if (message && isEnabled()) {
      rviz_default_plugins::displays::PolygonDisplay::processMessage(message);
    }
    rviz_default_plugins::displays::PolygonDisplay::update(wall_dt, ros_dt);
  }

  void reset() override
  {
    pending_.take();
    rviz_default_plugins::displays::PolygonDisplay::reset();
  }

protected:
  void processMessage(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) override
  {
    pending_.push(std::move(msg));
  }

private:
  LatestMessage<geometry_msgs::msg::PolygonStamped> pending_;
};

}  // namespace limo_rviz_plugins

PLUGINLIB_EXPORT_CLASS(limo_rviz_plugins::PolygonDisplay, rviz_common::Display)
