#include "gui/job_system.hpp"

#include <algorithm>
#include <exception>
#include <utility>

namespace kpt::gui {

struct JobSystem::Job {
  std::uint64_t id = 0;
  std::string name;
  JobPriority priority = JobPriority::Normal;
  Task task;
  std::stop_source stop;
  JobState state = JobState::Queued;
  float progress = 0.0F;
  std::string message;
};

bool JobSystem::Compare::operator()(const QueuedJob &lhs,
                                    const QueuedJob &rhs) const {
  if (lhs.priority != rhs.priority) {
    return static_cast<int>(lhs.priority) < static_cast<int>(rhs.priority);
  }
  return lhs.sequence > rhs.sequence;
}

JobSystem::JobSystem() {
  const unsigned detected = std::thread::hardware_concurrency();
  max_workers_ = std::max(1U, detected);
  worker_limit_.store((max_workers_ + 1U) / 2U);
  workers_.reserve(max_workers_);
  for (unsigned index = 0; index < max_workers_; ++index) {
    workers_.emplace_back(
        [this, index](std::stop_token stop) { workerLoop(stop, index); });
  }
}

JobSystem::~JobSystem() {
  {
    std::lock_guard lock(mutex_);
    for (auto &[id, job] : jobs_) {
      static_cast<void>(id);
      job->stop.request_stop();
    }
  }
  for (auto &worker : workers_)
    worker.request_stop();
  wake_.notify_all();
}

std::uint64_t JobSystem::submit(std::string name, JobPriority priority,
                                Task task) {
  auto job = std::make_shared<Job>();
  job->id = next_id_.fetch_add(1);
  job->name = std::move(name);
  job->priority = priority;
  job->task = std::move(task);

  {
    std::lock_guard lock(mutex_);
    jobs_[job->id] = job;
    queue_.push({priority, next_sequence_.fetch_add(1), job});
  }
  wake_.notify_all();
  return job->id;
}

void JobSystem::cancel(std::uint64_t id) {
  std::lock_guard lock(mutex_);
  const auto found = jobs_.find(id);
  if (found == jobs_.end())
    return;
  found->second->stop.request_stop();
  if (found->second->state == JobState::Queued) {
    found->second->state = JobState::Cancelled;
    found->second->message = "cancelled";
  }
  wake_.notify_all();
}

void JobSystem::cancelAll() {
  std::lock_guard lock(mutex_);
  for (auto &[id, job] : jobs_) {
    static_cast<void>(id);
    job->stop.request_stop();
    if (job->state == JobState::Queued) {
      job->state = JobState::Cancelled;
      job->message = "cancelled";
    }
  }
  wake_.notify_all();
}

void JobSystem::clearFinished() {
  std::lock_guard lock(mutex_);
  std::erase_if(jobs_, [](const auto &entry) {
    const auto state = entry.second->state;
    return state == JobState::Succeeded || state == JobState::Failed ||
           state == JobState::Cancelled;
  });
}

std::vector<JobSnapshot> JobSystem::snapshots() const {
  std::lock_guard lock(mutex_);
  std::vector<JobSnapshot> result;
  result.reserve(jobs_.size());
  for (const auto &[id, job] : jobs_) {
    result.push_back({id, job->name, job->priority, job->state, job->progress,
                      job->message});
  }
  std::sort(result.begin(), result.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.id > rhs.id; });
  return result;
}

void JobSystem::setWorkerLimit(unsigned limit) {
  worker_limit_.store(std::clamp(limit, 1U, max_workers_));
  wake_.notify_all();
}

void JobSystem::setPlayerActive(bool active) {
  player_active_.store(active);
  wake_.notify_all();
}

std::shared_ptr<JobSystem::Job> JobSystem::takeJob(unsigned worker_index) {
  const unsigned worker_limit = worker_limit_.load();
  if (worker_index >= worker_limit)
    return {};

  while (!queue_.empty()) {
    auto queued = queue_.top();
    queue_.pop();
    auto &job = queued.job;
    if (job->state != JobState::Queued || job->stop.stop_requested())
      continue;

    const bool reserved_worker =
        player_active_.load() && worker_limit > 1U &&
        worker_index == worker_limit - 1U;
    if (reserved_worker && job->priority != JobPriority::High) {
      queue_.push(std::move(queued));
      return {};
    }
    job->state = JobState::Running;
    job->message = "running";
    return job;
  }
  return {};
}

void JobSystem::workerLoop(std::stop_token stop, unsigned worker_index) {
  while (!stop.stop_requested()) {
    std::shared_ptr<Job> job;
    {
      std::unique_lock lock(mutex_);
      wake_.wait(lock, stop, [this, worker_index] {
        const unsigned worker_limit = worker_limit_.load();
        if (worker_index >= worker_limit || queue_.empty())
          return false;
        const bool reserved_worker =
            player_active_.load() && worker_limit > 1U &&
            worker_index == worker_limit - 1U;
        return !reserved_worker ||
               queue_.top().priority == JobPriority::High;
      });
      if (stop.stop_requested())
        return;
      job = takeJob(worker_index);
    }
    if (!job)
      continue;

    const Reporter reporter = [this, weak = std::weak_ptr<Job>(job)](
                                  float progress, std::string message) {
      if (const auto current = weak.lock()) {
        std::lock_guard lock(mutex_);
        current->progress = std::clamp(progress, 0.0F, 1.0F);
        current->message = std::move(message);
      }
    };

    try {
      job->task(job->stop.get_token(), reporter);
      std::lock_guard lock(mutex_);
      if (job->stop.stop_requested()) {
        job->state = JobState::Cancelled;
        job->message = "cancelled";
      } else {
        job->state = JobState::Succeeded;
        job->progress = 1.0F;
        if (job->message == "running")
          job->message = "done";
      }
      job->task = {};
    } catch (const std::exception &error) {
      std::lock_guard lock(mutex_);
      job->state =
          job->stop.stop_requested() ? JobState::Cancelled : JobState::Failed;
      job->message = error.what();
      job->task = {};
    } catch (...) {
      std::lock_guard lock(mutex_);
      job->state = JobState::Failed;
      job->message = "unknown error";
      job->task = {};
    }
  }
}

const char *jobStateName(JobState state) {
  switch (state) {
  case JobState::Queued:
    return "queued";
  case JobState::Running:
    return "running";
  case JobState::Succeeded:
    return "succeeded";
  case JobState::Failed:
    return "failed";
  case JobState::Cancelled:
    return "cancelled";
  }
  return "unknown";
}

} // namespace kpt::gui
