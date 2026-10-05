#ifndef BXI_HARDWARE__WRITE_SCHEDULE_HPP_
#define BXI_HARDWARE__WRITE_SCHEDULE_HPP_
#include <cstdint>
namespace bxi_hardware
{
// Pure timing policy, also used by the deterministic boundary tests.
struct WriteSchedule
{
  int64_t period_ns, grace_ns;
  uint64_t sequence = 0, sent_sequence = 0;
  int64_t published_ns = 0, last_send_ns = 0, resend_ns = 0;
  bool last_was_resend = false;
  void published(uint64_t seq, int64_t at)
  {
    sequence = seq;
    published_ns = at;
    resend_ns = at + period_ns + grace_ns;
  }
  bool pending() const {return sequence != sent_sequence;}
  int64_t earliestSend() const {return last_send_ns + period_ns / 2;}
  bool held(int64_t now) const {return last_send_ns != 0 && now < earliestSend();}
  bool resendDue(int64_t now) const {return !pending() && now >= resend_ns;}
  void sent(int64_t at, bool resend)
  {
    last_send_ns = at;
    last_was_resend = resend;
    sent_sequence = sequence;
    if (resend) {resend_ns = at + period_ns;}
  }
};
}  // namespace bxi_hardware
#endif
