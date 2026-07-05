#include "mmlp/thread_pool.hpp"

#include <algorithm>
#include <iostream>

namespace mmlp {

ThreadPool& ThreadPool::instance() {
  static ThreadPool pool;
  return pool;
}

ThreadPool::ThreadPool() {
  workers_ = 12;
  if (const char* env = std::getenv("MMLP_WORKERS")) {
    const int n = std::atoi(env);
    if (n > 0) {
      workers_ = static_cast<std::size_t>(n);
    }
  }
  workers_ = std::max<std::size_t>(1, workers_);
  std::cerr << "[mmlp] thread pool workers=" << workers_ << "\n" << std::flush;
  threads_.reserve(workers_);
  for (std::size_t i = 0; i < workers_; ++i) {
    threads_.emplace_back([this]() { workerLoop(); });
  }
}

ThreadPool::~ThreadPool() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  cv_.notify_all();
  for (auto& thread : threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

void ThreadPool::workerLoop() {
  while (true) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() { return stop_ || !tasks_.empty(); });
      if (stop_ && tasks_.empty()) {
        return;
      }
      task = std::move(tasks_.front());
      tasks_.pop();
    }
    task();
  }
}

void parallelFor(std::size_t count, const std::function<void(std::size_t)>& fn) {
  if (count == 0) {
    return;
  }
  if (count == 1) {
    fn(0);
    return;
  }

  ThreadPool& pool = ThreadPool::instance();
  const std::size_t nWorkers = std::min(count, pool.workers());
  if (nWorkers <= 1) {
    for (std::size_t i = 0; i < count; ++i) {
      fn(i);
    }
    return;
  }

  std::vector<std::future<void>> futures;
  futures.reserve(nWorkers);
  const std::size_t chunk = (count + nWorkers - 1) / nWorkers;
  for (std::size_t w = 0; w < nWorkers; ++w) {
    const std::size_t begin = w * chunk;
    const std::size_t end = std::min(count, begin + chunk);
    if (begin >= end) {
      break;
    }
    futures.push_back(pool.submit([begin, end, &fn]() {
      for (std::size_t i = begin; i < end; ++i) {
        fn(i);
      }
    }));
  }
  for (auto& future : futures) {
    future.get();
  }
}

}  // namespace mmlp
