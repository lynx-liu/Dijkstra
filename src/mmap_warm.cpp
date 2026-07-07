#include "mmlp/mmap_warm.hpp"

#include <algorithm>
#include <thread>
#include <vector>

namespace mmlp {

void warmMmapPages(const void* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return;
  }
  const char* bytes = static_cast<const char*>(data);
  constexpr std::size_t kStride = 256 * 1024;
  for (std::size_t off = 0; off < size; off += kStride) {
    (void)bytes[off];
  }
  (void)bytes[size - 1];
}

void warmMmapPagesAsync(const void* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return;
  }
  // Never use the routing thread pool for mmap warm — it can deadlock parallel routing.
  std::thread([data, size]() { warmMmapPages(data, size); }).detach();
}

void warmMmapPagesParallel(const void* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return;
  }
  constexpr std::size_t kParallelMin = 8 * 1024 * 1024;
  if (size < kParallelMin) {
    warmMmapPages(data, size);
    return;
  }

  const unsigned hw = std::thread::hardware_concurrency();
  std::size_t nWorkers = hw > 2 ? static_cast<std::size_t>(hw - 1) : std::max(1u, hw);
  nWorkers = std::min<std::size_t>(nWorkers, 8);
  const std::size_t chunk = (size + nWorkers - 1) / nWorkers;
  const char* bytes = static_cast<const char*>(data);
  std::vector<std::thread> threads;
  threads.reserve(nWorkers);
  for (std::size_t w = 0; w < nWorkers; ++w) {
    const std::size_t begin = w * chunk;
    if (begin >= size) {
      break;
    }
    const std::size_t end = std::min(size, begin + chunk);
    threads.emplace_back([bytes, begin, end]() {
      constexpr std::size_t kStride = 4096;
      for (std::size_t off = begin; off < end; off += kStride) {
        (void)bytes[off];
      }
      if (end > begin) {
        (void)bytes[end - 1];
      }
    });
  }
  for (auto& thread : threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

void warmMmapPagesParallelAsync(const void* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return;
  }
  std::thread([data, size]() { warmMmapPagesParallel(data, size); }).detach();
}

}  // namespace mmlp
