// Copyright 2026 Giulio Cataldo
// SPDX-License-Identifier: Apache-2.0

#ifndef LIMO_RVIZ_PLUGINS__LATEST_MESSAGE_HPP_
#define LIMO_RVIZ_PLUGINS__LATEST_MESSAGE_HPP_

#include <memory>
#include <mutex>
#include <utility>

namespace limo_rviz_plugins
{

// A stalled renderer must resume with the newest state, without replaying an
// accumulated history of footprint updates. ROS callbacks only replace a slot.
template<typename Message>
class LatestMessage
{
public:
  using Pointer = std::shared_ptr<const Message>;

  void push(Pointer message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = std::move(message);
  }

  Pointer take()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Pointer result;
    result.swap(pending_);
    return result;
  }

private:
  std::mutex mutex_;
  Pointer pending_;
};

}  // namespace limo_rviz_plugins

#endif  // LIMO_RVIZ_PLUGINS__LATEST_MESSAGE_HPP_
