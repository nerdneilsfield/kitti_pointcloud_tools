#include "gui/jobs/ui_events.hpp"

#include <utility>

namespace kpt::gui {

void UiEvents::post(std::function<void()> event) {
  std::lock_guard lock(mutex_);
  events_.push_back(std::move(event));
}

void UiEvents::drain() {
  std::deque<std::function<void()>> events;
  {
    std::lock_guard lock(mutex_);
    events.swap(events_);
  }
  for (auto &event : events)
    event();
}

} // namespace kpt::gui
