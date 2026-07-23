#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <stop_token>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace kpt::gui {

enum class JobPriority : int { Low = 0, Normal = 1, High = 2 };
enum class JobState { Queued, Running, Succeeded, Failed, Cancelled };

struct JobSnapshot {
  std::uint64_t id = 0;
  std::string name;
  JobPriority priority = JobPriority::Normal;
  JobState state = JobState::Queued;
  float progress = 0.0F;
  std::string message;
};

class JobSystem {
public:
  using Reporter = std::function<void(float, std::string)>;
  using Task = std::function<void(std::stop_token, const Reporter &)>;

  JobSystem();
  ~JobSystem();
  JobSystem(const JobSystem &) = delete;
  JobSystem &operator=(const JobSystem &) = delete;

  std::uint64_t submit(std::string name, JobPriority priority, Task task);
  void cancel(std::uint64_t id);
  void cancelAll();
  void clearFinished();

  [[nodiscard]] std::vector<JobSnapshot> snapshots() const;
  [[nodiscard]] unsigned maxWorkers() const { return max_workers_; }
  [[nodiscard]] unsigned workerLimit() const { return worker_limit_.load(); }
  void setWorkerLimit(unsigned limit);
  void setPlayerActive(bool active);

private:
  struct Job;
  struct QueuedJob {
    JobPriority priority;
    std::uint64_t sequence;
    std::shared_ptr<Job> job;
  };
  struct Compare {
    bool operator()(const QueuedJob &lhs, const QueuedJob &rhs) const;
  };

  void workerLoop(std::stop_token stop, unsigned worker_index);
  std::shared_ptr<Job> takeJob(unsigned worker_index);

  unsigned max_workers_ = 1;
  std::atomic<unsigned> worker_limit_{1};
  std::atomic<bool> player_active_{false};
  std::atomic<std::uint64_t> next_id_{1};
  std::atomic<std::uint64_t> next_sequence_{1};

  mutable std::mutex mutex_;
  std::condition_variable_any wake_;
  std::priority_queue<QueuedJob, std::vector<QueuedJob>, Compare> queue_;
  std::unordered_map<std::uint64_t, std::shared_ptr<Job>> jobs_;
  std::vector<std::jthread> workers_;
};

const char *jobStateName(JobState state);

} // namespace kpt::gui
