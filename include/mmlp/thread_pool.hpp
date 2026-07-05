#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace mmlp {

class ThreadPool {
 public:
  static ThreadPool& instance();

  std::size_t workers() const { return workers_; }

  template <typename F, typename... Args>
  auto submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))>;

 private:
  friend void parallelFor(std::size_t count, const std::function<void(std::size_t)>& fn);

  ThreadPool();
  ~ThreadPool();

  void workerLoop();

  std::vector<std::thread> threads_;
  std::queue<std::function<void()>> tasks_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_ = false;
  std::size_t workers_ = 1;
};

template <typename F, typename... Args>
auto ThreadPool::submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))> {
  using Ret = decltype(f(args...));
  auto bound = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
  auto task = std::make_shared<std::packaged_task<Ret()>>(std::move(bound));
  std::future<Ret> result = task->get_future();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tasks_.push([task]() { (*task)(); });
  }
  cv_.notify_one();
  return result;
}

void parallelFor(std::size_t count, const std::function<void(std::size_t)>& fn);

}  // namespace mmlp
