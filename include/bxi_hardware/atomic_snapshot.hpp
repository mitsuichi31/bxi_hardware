#ifndef BXI_HARDWARE__ATOMIC_SNAPSHOT_HPP_
#define BXI_HARDWARE__ATOMIC_SNAPSHOT_HPP_
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <type_traits>

namespace bxi_hardware
{
// One writer, fixed capacity, no fences or non-atomic concurrent payload accesses.
// All operations participate in the same SC order. An unchanged even version means
// no writer's payload stores occurred between the reader's two version loads.
// The 64-bit version must not wrap during a read (2^63 publications).
// Readers use separate scratch storage: a failed read must not replace their cache.
template<class T>
class AtomicSnapshot
{
  static_assert(std::is_trivially_copyable<T>::value, "snapshot payload must be trivially copyable");
  static constexpr size_t words = (sizeof(T) + 7) / 8;
public:
  explicit AtomicSnapshot(size_t count = 1)
  : count_(count), data_(std::make_unique<std::atomic<uint64_t>[]>(count * words))
  {
    if (!count) {throw std::invalid_argument("snapshot count must be positive");}
    for (size_t i = 0; i < count * words; ++i) {data_[i].store(0);}
  }
  bool lockFree() const {return version_.is_lock_free() && data_[0].is_lock_free();}
  uint64_t version() const {return version_.load();}
  void publish(const T * values)
  {
    const auto v = version_.load();
    version_.store(v + 1);
    for (size_t i = 0; i < count_; ++i) {
      for (size_t w = 0; w < words; ++w) {
        uint64_t bits = 0;
        const auto bytes = std::min(size_t{8}, sizeof(T) - w * 8);
        std::memcpy(&bits, reinterpret_cast<const char *>(&values[i]) + w * 8, bytes);
        data_[i * words + w].store(bits);
      }
    }
    version_.store(v + 2);
  }
  bool tryRead(T * scratch, unsigned * attempts = nullptr) const
  {
    for (unsigned attempt = 1; attempt <= 2; ++attempt) {
      if (attempts) {*attempts = attempt;}
      const auto before = version_.load();
      if (before == 0 || (before & 1)) {continue;}
      for (size_t i = 0; i < count_; ++i) {
        for (size_t w = 0; w < words; ++w) {
          const auto bits = data_[i * words + w].load();
          const auto bytes = std::min(size_t{8}, sizeof(T) - w * 8);
          std::memcpy(reinterpret_cast<char *>(&scratch[i]) + w * 8, &bits, bytes);
        }
      }
      if (version_.load() == before) {return true;}
    }
    return false;
  }
private:
  friend struct AtomicSnapshotTestAccess;
  size_t count_;
  std::unique_ptr<std::atomic<uint64_t>[]> data_;
  std::atomic<uint64_t> version_{0};
};
}  // namespace bxi_hardware
#endif
