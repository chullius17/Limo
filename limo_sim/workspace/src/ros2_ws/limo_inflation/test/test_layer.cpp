#include <gtest/gtest.h>

#include <memory>

#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_loader.hpp"

TEST(BorderFollowLayerTest, IsDiscoverableByPluginlib)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d",
    "nav2_costmap_2d::Layer");

  auto layer = loader.createSharedInstance(
    "limo_inflation::BorderFollowLayer");
  ASSERT_NE(layer, nullptr);
}

TEST(LocalLaneLayerTest, IsDiscoverableByPluginlib)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("limo_inflation::LocalLaneLayer");
  ASSERT_NE(layer, nullptr);
}

TEST(GaussianLayerTest, IsDiscoverableByPluginlib)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("limo_inflation::GaussianLayer");
  ASSERT_NE(layer, nullptr);
}

TEST(PathDistanceLayerTest, IsDiscoverableByPluginlib)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  ASSERT_NE(loader.createSharedInstance("limo_inflation::PathDistanceLayer"), nullptr);
}
