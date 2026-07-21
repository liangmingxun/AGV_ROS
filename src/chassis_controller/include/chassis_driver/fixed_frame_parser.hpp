#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace chassis_driver {

class FixedFrameParser {
 public:
  static constexpr std::size_t kPayloadSize = 60;
  static constexpr std::size_t kMarkerSize = 3;
  static constexpr std::size_t kFrameSize =
      kMarkerSize + kPayloadSize + kMarkerSize;

  void append(const std::uint8_t* data, std::size_t size) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  bool popPayload(std::vector<std::uint8_t>& payload, bool* resynchronised = nullptr) {
    if (resynchronised) *resynchronised = false;
    while (true) {
      const auto start = std::search(buffer_.begin(), buffer_.end(),
                                     kFrameStart.begin(), kFrameStart.end());
      if (start == buffer_.end()) {
        const std::size_t keep = matchingStartSuffix();
        if (buffer_.size() > keep) {
          buffer_.erase(buffer_.begin(), buffer_.end() - keep);
          if (resynchronised) *resynchronised = true;
        }
        return false;
      }

      if (start != buffer_.begin()) {
        buffer_.erase(buffer_.begin(), start);
        if (resynchronised) *resynchronised = true;
      }
      if (buffer_.size() < kFrameSize) return false;

      const auto terminator = buffer_.begin() + kMarkerSize + kPayloadSize;
      if (std::equal(kFrameEnd.begin(), kFrameEnd.end(), terminator)) {
        payload.assign(buffer_.begin() + kMarkerSize, terminator);
        buffer_.erase(buffer_.begin(), buffer_.begin() + kFrameSize);
        return true;
      }

      buffer_.pop_front();
      if (resynchronised) *resynchronised = true;
    }
  }

 private:
  std::size_t matchingStartSuffix() const {
    const std::size_t maximum =
        std::min(buffer_.size(), kFrameStart.size() - 1);
    for (std::size_t length = maximum; length > 0; --length) {
      if (std::equal(buffer_.end() - length, buffer_.end(),
                     kFrameStart.begin())) {
        return length;
      }
    }
    return 0;
  }

  inline static constexpr std::array<std::uint8_t, kMarkerSize> kFrameStart{
      '#', '$', '#'};
  inline static constexpr std::array<std::uint8_t, kMarkerSize> kFrameEnd{
      '!', '@', '!'};
  std::deque<std::uint8_t> buffer_;
};

}  // namespace chassis_driver
