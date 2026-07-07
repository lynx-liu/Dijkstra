#pragma once

#include <cstddef>

namespace mmlp {

// Touch mapped pages so the OS loads them into page cache / RAM.
void warmMmapPages(const void* data, std::size_t size);

// Background warm via the global thread pool (no-op if data is null).
void warmMmapPagesAsync(const void* data, std::size_t size);

// Parallel page walk for large mmap regions (uses all thread-pool workers).
void warmMmapPagesParallel(const void* data, std::size_t size);

// Background parallel warm (preferred for regional CSR at startup).
void warmMmapPagesParallelAsync(const void* data, std::size_t size);

}  // namespace mmlp
