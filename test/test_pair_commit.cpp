#include <gtest/gtest.h>
#include <limits>
#include "bxi_hardware/pair_commit.hpp"
#include "bxi_hardware/atomic_snapshot.hpp"
#include "bxi_hardware/bus_streamer.hpp"
#include "bxi_hardware/write_schedule.hpp"
namespace bxi_hardware
{
struct PairCommitTestAccess {static void last(PairCommit & pair, uint64_t last) {pair.last_ = last;}};
namespace
{
using R = PairCommit::Reason;
constexpr int64_t a0 = 1000000000;
PairCommit makePair(int64_t timeout = 20000000)
{
  PairCommit pair;
  PairCommit::Bounds bounds;
  for (size_t i = 0; i < 10; ++i) {
    bounds.lower[i] = i % 5 < 2 ? -1 : 0;
    bounds.upper[i] = 1;
  }
  pair.bounds(bounds); pair.reset(a0, timeout);
  return pair;
}
TEST(PairCommit, StrictNextAndDuplicateDoesNotConsumeGeneration)
{
  auto pair = makePair(); PairCommit::Values values{};
  EXPECT_EQ(pair.inspect(0, values, a0), R::kBootWait);
  ASSERT_EQ(pair.inspect(1, values, a0), R::kAccepted); pair.accept(1);
  values[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(pair.inspect(1, values, a0), R::kUnchanged);
  EXPECT_EQ(pair.last(), 1U); values[0] = 0;
  EXPECT_EQ(pair.inspect(2, values, a0), R::kAccepted);
}
TEST(PairCommit, InvalidGenerationBeforeIntegerCast)
{
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
      -1., 1.5, 9007199254740992.}) {
    auto pair = makePair();
    EXPECT_EQ(pair.inspect(value, {}, a0), R::kInvalidGeneration);
    EXPECT_TRUE(pair.latched()); EXPECT_EQ(pair.last(), 0U);
  }
}
TEST(PairCommit, RollbackZeroAndGapLatch)
{
  for (double value : {0., 3.}) {
    auto pair = makePair(); pair.accept(1);
    EXPECT_EQ(pair.inspect(value, {}, a0), value == 0 ? R::kRollback : R::kGap);
    EXPECT_TRUE(pair.latched()); EXPECT_EQ(pair.last(), 1U);
  }
}
TEST(PairCommit, AllTenFieldsRejectNonfiniteAndOutOfRange)
{
  for (size_t field = 0; field < 10; ++field) {
    for (double value : {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -2., 2.}) {
      auto pair = makePair(); pair.accept(1); PairCommit::Values values{}; values[field] = value;
      EXPECT_EQ(pair.inspect(2, values, a0), R::kInvalidBatch) << field;
      EXPECT_EQ(pair.last(), 1U); EXPECT_TRUE(pair.latched());
    }
  }
}
TEST(PairCommit, LatchedOnlyAcceptsReleasedFiniteBatchWithoutClearingLatch)
{
  auto pair = makePair(); pair.accept(1); pair.latch();
  for (size_t field = 0; field < 10; ++field) {
    if (field % 5 == 0) {continue;}
    PairCommit::Values values{}; values[field] = .5;
    EXPECT_EQ(pair.inspect(2, values, a0), R::kLatchedNonzero);
  }
  EXPECT_EQ(pair.inspect(2, {}, a0), R::kAccepted); pair.accept(2);
  EXPECT_TRUE(pair.latched());
}
TEST(PairCommit, FirstCommitMustReleaseBothAxes)
{
  auto pair = makePair(); PairCommit::Values values{}; values[7] = .5;
  EXPECT_EQ(pair.inspect(1, values, a0), R::kLatchedNonzero);
}
TEST(PairCommit, LastExactGenerationAndTwoTo53Rejected)
{
  auto pair = makePair(); PairCommitTestAccess::last(pair, PairCommit::kMaxGeneration - 1);
  EXPECT_EQ(pair.inspect(static_cast<double>(PairCommit::kMaxGeneration), {}, a0), R::kAccepted);
  pair.accept(static_cast<double>(PairCommit::kMaxGeneration));
  EXPECT_EQ(pair.inspect(9007199254740992., {}, a0), R::kUpperLimit);
  EXPECT_EQ(pair.last(), PairCommit::kMaxGeneration);
}
TEST(PairCommit, FixedBootstrapDeadlineBeforeEqualAfterBothTimeouts)
{
  for (int64_t timeout : {20000000LL, 1000000000LL}) {
    for (int64_t delta : {-1LL, 0LL, 1LL}) {
      auto pair = makePair(timeout);
      EXPECT_EQ(pair.inspect(1, {}, a0 + timeout + delta), delta < 0 ? R::kAccepted : R::kStartupExpired);
      EXPECT_EQ(pair.deadline(), a0 + timeout);
    }
  }
}
TEST(PairCommit, StageZeroCannotExtendDeadlineOrSupplyBootstrap)
{
  auto pair = makePair(); PairCommit::Values values; values.fill(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(pair.inspect(0, values, a0 + 100), R::kBootWait);
  values.fill(1);
  EXPECT_EQ(pair.inspect(0, values, a0 + 1000), R::kBootWait);
  EXPECT_EQ(pair.deadline(), a0 + 20000000);
  EXPECT_EQ(pair.inspect(0, values, pair.deadline()), R::kStartupExpired);
  EXPECT_EQ(pair.inspect(1, {}, a0), R::kStartupExpired);
}
TEST(PairCommit, CommittedBeforeInitialDeadlineRemainsValidWhenTxReadsLate)
{
  auto pair = makePair();
  EXPECT_EQ(pair.inspect(1, {}, pair.deadline() - 1), R::kAccepted); pair.accept(1);
  EXPECT_EQ(pair.inspect(1, {}, pair.deadline() + 1), R::kUnchanged);
}
TEST(PairCommit, PublishedTimestampIsNotTxReadTimestamp)
{
  AtomicSnapshot<PublishedCommand> snapshot(2);
  std::array<PublishedCommand, 2> packets{}, read{};
  for (auto & p : packets) {p.sequence = 1; p.published_ns = a0;}
  snapshot.publish(packets.data());
  for (auto & p : packets) {p.sequence = 2; p.published_ns = a0 + 1000;}
  snapshot.publish(packets.data());
  ASSERT_TRUE(snapshot.tryRead(read.data()));
  EXPECT_EQ(read[0].sequence, 2U); EXPECT_EQ(read[1].sequence, 2U);
  EXPECT_EQ(read[0].published_ns, a0 + 1000); EXPECT_EQ(read[1].published_ns, a0 + 1000);
}
}  // namespace
}  // namespace bxi_hardware
