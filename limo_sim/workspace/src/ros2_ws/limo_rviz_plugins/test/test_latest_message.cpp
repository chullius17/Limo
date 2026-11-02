// Copyright 2026 Giulio Cataldo
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "limo_rviz_plugins/latest_message.hpp"

TEST(LatestMessage, StalledRendererRetainsOnlyNewestFootprint)
{
  limo_rviz_plugins::LatestMessage<int> slot;
  std::weak_ptr<const int> oldest;
  for (int i = 0; i < 100000; ++i) {
    auto message = std::make_shared<const int>(i);
    if (i == 0) {
      oldest = message;
    }
    slot.push(message);
  }
  EXPECT_TRUE(oldest.expired());
  auto message = slot.take();
  ASSERT_TRUE(message);
  EXPECT_EQ(*message, 99999);
  EXPECT_FALSE(slot.take());
}

TEST(LatestMessage, ConcurrentCallbackAndRendererNeverReplayOlderState)
{
  limo_rviz_plugins::LatestMessage<int> slot;
  std::atomic<bool> finished{false};
  std::thread callback([&]() {
    for (int i = 0; i < 100000; ++i) {
      slot.push(std::make_shared<const int>(i));
    }
    finished.store(true);
  });
  int last = -1;
  while (!finished.load()) {
    if (auto message = slot.take()) {
      EXPECT_GT(*message, last);
      last = *message;
    }
  }
  callback.join();
  if (auto message = slot.take()) {
    EXPECT_GT(*message, last);
    last = *message;
  }
  EXPECT_EQ(last, 99999);
}
