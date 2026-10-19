#ifndef BXI_HARDWARE__ACTIVATION_STARTUP_HPP_
#define BXI_HARDWARE__ACTIVATION_STARTUP_HPP_

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>
#include "bxi_hardware/actuator/bxi_actuator.hpp"

namespace bxi_hardware
{
inline const char * kernelVsSend(int64_t kernel_realtime, int64_t send_realtime)
{
  return !kernel_realtime || !send_realtime ? "NOT_EVALUATED" :
         (kernel_realtime < send_realtime ? "before" : "at_or_after");
}

inline bool initialPositionAllowed(double p, double lo, double hi) noexcept
{
  return std::isfinite(p) && std::isfinite(lo) && std::isfinite(hi) && lo <= hi && p >= lo &&
         p <= hi;
}

// All provisional startup policy values live here. Never used by the RT paths.
struct ActivationPolicy
{
  static constexpr int64_t settle_ns = 10000000;
  static constexpr int attempts = 2;
  static constexpr int64_t retry_interval_ns = 5000000;
  static constexpr int64_t budget_ns = 100000000;
};
struct ActivationClock
{
  std::function<int64_t()> now;
  std::function<void(int64_t)> wait_until;
};
struct InitialAttempt
{
  int number = 0;
  TransactionObservation observation;
  const char * reason = "exception";
};
// This object and every callback belong to the activation caller's thread.
struct ActivationStartup
{
  ActivationClock clock;
  int64_t deadline;
  int64_t response_timeout;
  int64_t settle_end = 0;
  int64_t settle_wait = 0;
  const char * reason = "deadline";

  bool room(int64_t wait = 0) const
  {
    const auto now = clock.now();
    return now < deadline && wait <= deadline - now &&
           response_timeout <= deadline - now - wait;
  }

  bool settle(int64_t last_enable)
  {
    const auto before = clock.now();
    const auto target = last_enable + ActivationPolicy::settle_ns;
    if (!room(std::max<int64_t>(0, target - before))) {return false;}
    if (before < target) {clock.wait_until(target);}
    settle_end = clock.now();
    settle_wait = settle_end - before;
    return settle_end >= target && room();
  }

  bool initial(
    const std::function<ActuatorFeedback(TransactionObservation &)> & send,
    const std::function<bool()> & enabled, ActuatorFeedback & feedback,
    std::vector<InitialAttempt> & records)
  {
    for (int attempt = 1; attempt <= ActivationPolicy::attempts; ++attempt) {
      if (!enabled()) {reason = "safe_stop"; return false;}
      if (!room()) {reason = "deadline"; return false;}
      records.push_back({attempt, {}, "exception"});
      auto & record = records.back();
      if (!room()) {record.reason = reason = "deadline"; return false;}
      try {feedback = send(record.observation);} catch (...) {reason = "exception"; throw;}
      record.reason = record.observation.result;
      if (clock.now() >= deadline) {record.reason = reason = "deadline"; return false;}
      if (feedback.valid) {
        if (!std::isfinite(feedback.temperature)) {
          record.reason = reason = "temperature_fault";
          return false;
        }
        record.reason = reason = "ok";
        return true;
      }
      if (!enabled() || std::string(record.observation.result) == "send_failed") {
        reason = std::string(record.observation.result) ==
          "send_failed" ? "send_failed" : "safe_stop";
        record.reason = reason;
        return false;
      }
      if (std::string(record.observation.result) != "timeout") {
        record.reason = reason = record.observation.result;
        return false;
      }
      if (attempt == ActivationPolicy::attempts) {
        record.reason = reason = "attempt_limit";
        return false;
      }
      if (!room(ActivationPolicy::retry_interval_ns)) {reason = "deadline"; return false;}
      const auto target = clock.now() + ActivationPolicy::retry_interval_ns;
      clock.wait_until(target);
      if (clock.now() < target || !room()) {reason = "deadline"; return false;}
    }
    return false;
  }

};
}  // namespace bxi_hardware
#endif
