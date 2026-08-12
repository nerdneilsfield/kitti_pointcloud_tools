#pragma once

#include "kpt/types.hpp"

#include <algorithm>
#include <cstddef>
#include <deque>
#include <unordered_map>
#include <unordered_set>

namespace kpt::gui {

// Bounded LRU plus in-flight set for sequence frames. Keeping both collections
// behind one contract prevents an async completion from updating only half of
// App's former three-container cache state.
class SequenceFrameCache {
public:
  explicit SequenceFrameCache(std::size_t capacity = 3)
      : capacity_(capacity) {}

  void clear() noexcept {
    clouds_.clear();
    order_.clear();
    pending_.clear();
  }

  [[nodiscard]] bool empty() const noexcept {
    return clouds_.empty() && pending_.empty();
  }
  [[nodiscard]] bool cacheEmpty() const noexcept { return clouds_.empty(); }
  [[nodiscard]] bool orderEmpty() const noexcept { return order_.empty(); }
  [[nodiscard]] bool pendingEmpty() const noexcept { return pending_.empty(); }
  [[nodiscard]] bool isPending(std::size_t index) const noexcept {
    return pending_.contains(index);
  }

  [[nodiscard]] bool begin(std::size_t index) {
    return pending_.insert(index).second;
  }
  void finish(std::size_t index) noexcept { pending_.erase(index); }

  [[nodiscard]] PointCloudIRGBConstPtr find(std::size_t index) {
    const auto found = clouds_.find(index);
    if (found == clouds_.end())
      return {};
    std::erase(order_, index);
    order_.push_back(index);
    return found->second;
  }

  void store(std::size_t index, PointCloudIRGBConstPtr cloud,
             std::size_t protected_current, std::size_t protected_desired) {
    clouds_[index] = std::move(cloud);
    std::erase(order_, index);
    order_.push_back(index);
    while (clouds_.size() > capacity_) {
      const auto victim = std::ranges::find_if(
          order_, [protected_current, protected_desired](std::size_t candidate) {
            return candidate != protected_current &&
                   candidate != protected_desired;
          });
      if (victim == order_.end())
        break;
      clouds_.erase(*victim);
      order_.erase(victim);
    }
  }

private:
  std::size_t capacity_;
  std::unordered_map<std::size_t, PointCloudIRGBConstPtr> clouds_;
  std::deque<std::size_t> order_;
  std::unordered_set<std::size_t> pending_;
};

} // namespace kpt::gui
