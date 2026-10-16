#ifndef BXI_HARDWARE__PAIR_COMMIT_HPP_
#define BXI_HARDWARE__PAIR_COMMIT_HPP_

#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>

namespace bxi_hardware
{
// Stored, post-limiter values only. Neither setter success nor the writer's identity is observable.
class PairCommit
{
public:
  static constexpr uint64_t kMaxGeneration = 9007199254740991ULL;
  static constexpr size_t kMaxAxes = 3;
  static constexpr size_t kFieldsPerAxis = 5;
  static constexpr size_t kMaxValues = kMaxAxes * kFieldsPerAxis;
  using Values = std::array<double, kMaxValues>;
  // Set in non-RT configure; invalid sizes leave the existing configuration unchanged.
  bool axes(size_t count) noexcept {
    if (count < 2 || count > kMaxAxes) {return false;}
    axes_ = count; return true;
  }
  size_t valueCount() const noexcept {return axes_ * kFieldsPerAxis;}
  enum class Reason : uint8_t
  {
    kBootWait, kUnchanged, kAccepted, kInvalidGeneration, kRollback, kGap,
    kInvalidBatch, kLatchedNonzero, kStartupExpired, kUpperLimit, kPublishFailed, kNotClaimed, kFeedbackPending
  };
  struct Bounds {Values lower{}, upper{};};
  void reset(int64_t activation_ns, int64_t timeout_ns) noexcept
  {
    last_ = 0; latched_ = false; expired_ = false;
    deadline_ = activation_ns + timeout_ns;
  }
  void bounds(const Bounds & value) noexcept {bounds_ = value;}
  uint64_t last() const noexcept {return last_;}
  bool latched() const noexcept {return latched_;}
  int64_t deadline() const noexcept {return deadline_;}
  void latch() noexcept {latched_ = true;}
  Reason inspect(double generation, const Values & values, int64_t now) noexcept
  {
    if (!last_ && now >= deadline_) {expired_ = latched_ = true;}
    if (expired_) {return Reason::kStartupExpired;}
    if (!std::isfinite(generation) || generation < 0 ||
      generation > static_cast<double>(kMaxGeneration) || std::floor(generation) != generation)
    {
      latched_ = true;
      return last_ == kMaxGeneration ? Reason::kUpperLimit : Reason::kInvalidGeneration;
    }
    const auto g = static_cast<uint64_t>(generation);
    if (!g && !last_) {return Reason::kBootWait;}
    if (g == last_) {return Reason::kUnchanged;}
    if (g < last_) {latched_ = true; return Reason::kRollback;}
    if (last_ == kMaxGeneration) {return Reason::kUpperLimit;}
    if (g != last_ + 1) {latched_ = true; return Reason::kGap;}
    for (size_t i = 0; i < valueCount(); ++i) {
      if (!std::isfinite(values[i]) || values[i] < bounds_.lower[i] ||
        values[i] > bounds_.upper[i])
      {latched_ = true; return Reason::kInvalidBatch;}
    }
    // The first committed batch and every batch after a fault must be released.
    if (latched_ || !last_) {
      for (size_t i = 0; i < valueCount(); ++i) {
        if (i % 5 != 0 && values[i] != 0.0) {
          latched_ = true; return Reason::kLatchedNonzero;
        }
      }
    }
    return Reason::kAccepted;
  }
  // Called only once the complete batch has its publication timestamp, before publishing.
  void accept(double generation) noexcept {last_ = static_cast<uint64_t>(generation);}
private:
  friend struct PairCommitTestAccess;
  size_t axes_ = 2;
  Bounds bounds_{};
  uint64_t last_ = 0;
  int64_t deadline_ = 0;
  bool latched_ = false, expired_ = false;
};
}  // namespace bxi_hardware
#endif  // BXI_HARDWARE__PAIR_COMMIT_HPP_
