#pragma once

#include <deque>
#include <functional>
#include <mutex>

namespace kpt::gui {

class UiEvents {
public:
  void post(std::function<void()> event);
  void drain();

private:
  std::mutex mutex_;
  std::deque<std::function<void()>> events_;
};

} // namespace kpt::gui
