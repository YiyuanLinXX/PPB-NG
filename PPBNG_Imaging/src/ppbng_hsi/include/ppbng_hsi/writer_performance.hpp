#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ostream>

namespace ppbng_hsi
{
struct LatencyCounter
{
  std::uint64_t count{0}, total_us{0}, last_us{0}, max_us{0}, slow_count{0};
  void observe(std::uint64_t us) noexcept
  {
    ++count;
    total_us += us;
    last_us = us;
    max_us = (std::max)(max_us, us);
    if (us >= 100000U) ++slow_count;
  }
};

// Measures wall time (including scheduling/I/O waits), not CPU time. No allocation.
class ScopedLatency
{
public:
  explicit ScopedLatency(LatencyCounter & counter) : counter_(counter) {}
  ~ScopedLatency()
  {
    counter_.observe(static_cast<std::uint64_t>(std::chrono::duration_cast<
      std::chrono::microseconds>(std::chrono::steady_clock::now() - begin_).count()));
  }
  ScopedLatency(const ScopedLatency &) = delete;
  ScopedLatency & operator=(const ScopedLatency &) = delete;
private:
  LatencyCounter & counter_;
  std::chrono::steady_clock::time_point begin_{std::chrono::steady_clock::now()};
};

struct WriterPerformance
{
  LatencyCounter append, crc, raw_write, sidecar_write, stream_flush, checkpoint, rollover;
};

inline void write_counter_json(std::ostream & out, const LatencyCounter & counter)
{
  out << "{\"count\":" << counter.count << ",\"total_us\":" << counter.total_us
      << ",\"last_us\":" << counter.last_us << ",\"max_us\":" << counter.max_us
      << ",\"slow_count\":" << counter.slow_count << '}';
}

inline void write_performance_json(std::ostream & out, const WriterPerformance & p)
{
  out << "{\"append\":"; write_counter_json(out, p.append);
  out << ",\"crc\":"; write_counter_json(out, p.crc);
  out << ",\"raw_write\":"; write_counter_json(out, p.raw_write);
  out << ",\"sidecar_write\":"; write_counter_json(out, p.sidecar_write);
  out << ",\"stream_flush\":"; write_counter_json(out, p.stream_flush);
  out << ",\"checkpoint\":"; write_counter_json(out, p.checkpoint);
  out << ",\"rollover\":"; write_counter_json(out, p.rollover);
  out << '}';
}
}  // namespace ppbng_hsi
