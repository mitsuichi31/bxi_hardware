#include <array>
#include <atomic>
#include <thread>
#include <gtest/gtest.h>
#include "bxi_hardware/atomic_snapshot.hpp"
#include "bxi_hardware/write_schedule.hpp"
namespace bxi_hardware
{
struct AtomicSnapshotTestAccess
{
  template<class T> static void pauseWriter(AtomicSnapshot<T> & value)
  {value.version_.store(value.version_.load() + 1);}
};
TEST(AtomicSnapshot, PausedWriterCannotMakeTheReaderSpin)
{
  AtomicSnapshot<uint64_t> value;
  const uint64_t initial = 123;
  value.publish(&initial);
  AtomicSnapshotTestAccess::pauseWriter(value);
  uint64_t scratch = 0;
  unsigned attempts = 0;
  EXPECT_FALSE(value.tryRead(&scratch, &attempts));
  EXPECT_EQ(attempts, 2U);
}
TEST(AtomicSnapshot, EmptySnapshotDoesNotInventFeedback)
{
  AtomicSnapshot<std::array<uint64_t, 16>> snapshot;
  std::array<uint64_t, 16> result{};
  unsigned attempts = 0;
  EXPECT_TRUE(snapshot.lockFree());
  EXPECT_FALSE(snapshot.tryRead(&result, &attempts));
  EXPECT_EQ(attempts, 2U);
}
TEST(AtomicSnapshot, ConcurrentWholeBatchNeverMixesGenerations)
{
  using Value = std::array<uint64_t, 16>;
  AtomicSnapshot<Value> snapshot(3);
  std::atomic<bool> done{false};
  std::thread writer([&] {
    Value values[3];
    for (uint64_t generation = 1; generation <= 10000; ++generation) {
      for (auto & value : values) {value.fill(generation);}
      snapshot.publish(values);
    }
    done = true;
  });
  size_t accepted = 0;
  do {
    Value scratch[3];
    unsigned attempts = 0;
    if (snapshot.tryRead(scratch, &attempts)) {
      ++accepted;
      for (const auto & value : scratch) {
        for (auto field : value) {EXPECT_EQ(field, scratch[0][0]);}
      }
    }
    EXPECT_LE(attempts, 2U);
  } while (!done);
  writer.join();
  Value final[3];
  ASSERT_TRUE(snapshot.tryRead(final));
  EXPECT_EQ(final[2][15], 10000U);
  EXPECT_GT(accepted + 1, 0U);
}
TEST(WriteSchedule, GraceAvoidsNormalWriteResendCollision)
{
  WriteSchedule policy{1000000, 500000};
  policy.published(1, 10000000);
  policy.sent(10050000, false);
  EXPECT_FALSE(policy.resendDue(11000000));
  policy.published(2, 11100000);
  EXPECT_TRUE(policy.pending());
  EXPECT_FALSE(policy.resendDue(11500000));
  policy.sent(11150000, false);
  EXPECT_FALSE(policy.resendDue(12599999));
  EXPECT_TRUE(policy.resendDue(12600000));
}
TEST(WriteSchedule, ResendsAndHalfPeriodBoundary)
{
  WriteSchedule policy{1000000, 500000};
  policy.published(1, 10000000);
  policy.sent(10000000, false);
  for (int i = 0; i < 4; ++i) {
    const auto due = 11500000 + i * 1000000;
    EXPECT_FALSE(policy.resendDue(due - 1));
    EXPECT_TRUE(policy.resendDue(due));
    policy.sent(due, true);
    EXPECT_TRUE(policy.held(due + 499999));
    EXPECT_FALSE(policy.held(due + 500000));
  }
}
}  // namespace bxi_hardware
