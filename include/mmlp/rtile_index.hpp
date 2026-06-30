#pragma once

#include "mmlp/bbox.hpp"

#include <cstdint>
#include <string>
#include <unordered_set>

namespace mmlp {

// mmap partition tile edge-id cache (0.25° cells). Built by mmlp_build_rtiles.
class RtileIndex {
 public:
  RtileIndex() = default;
  ~RtileIndex();

  RtileIndex(const RtileIndex&) = delete;
  RtileIndex& operator=(const RtileIndex&) = delete;

  bool open(const std::string& basePath, std::string* error = nullptr);
  bool isOpen() const { return data_ != nullptr; }
  std::size_t tileCount() const { return tileCount_; }

  void collectEdgesInBBox(const GeoBBox& box, std::unordered_set<int64_t>& edgeIds) const;

 private:
  void* data_ = nullptr;
  std::size_t size_ = 0;
  std::size_t tileCount_ = 0;
  const void* entries_ = nullptr;
  const int64_t* edgeData_ = nullptr;
};

}  // namespace mmlp
