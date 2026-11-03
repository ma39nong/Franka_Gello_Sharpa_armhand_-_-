#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <string>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <queue>
#include <utility>

namespace orbbec_camera {

// No SDK calls, wall-clock dependence, allocation or logging on the frame path.
struct FrameStreamHealth {
  std::atomic<int64_t> last_sdk_us{0}, last_publish_us{0}, demand_since_us{0};
  std::atomic<uint64_t> sdk_count{0}, publish_count{0}, sdk_index{0}, publish_index{0};
  std::atomic<uint64_t> sdk_gap_total{0}, publish_gap_total{0}, queue_drop_total{0};
  std::atomic<int64_t> last_queue_drop_us{0}, decode_us{0}, processing_us{0}, queue_wait_us{0};
  std::atomic<int64_t> max_decode_us{0}, max_processing_us{0}, max_queue_wait_us{0};
  std::atomic<size_t> queue_size{0}, queue_high_watermark{0};
  static void maximum(std::atomic<int64_t> &value, int64_t sample) {
    auto old = value.load();
    while (old < sample && !value.compare_exchange_weak(old, sample)) {}
  }
  void arrival(int64_t now_us, uint64_t index, int64_t gap_us) {
    auto last = last_sdk_us.exchange(now_us);
    if (last && now_us-last > gap_us) ++sdk_gap_total;
    sdk_index = index; ++sdk_count;
  }
  void published(int64_t now_us, uint64_t index, int64_t gap_us) {
    auto last = last_publish_us.exchange(now_us);
    const auto since = demand_since_us.load();
    if (last && since && last >= since && now_us-last > gap_us) ++publish_gap_total;
    publish_index = index; ++publish_count;
  }
  void demand(bool enabled, int64_t now_us) {
    if (!enabled) demand_since_us = 0;
    else { int64_t zero = 0; demand_since_us.compare_exchange_strong(zero, now_us); }
  }
  std::string state(int64_t now_us, int64_t pipeline_start_us, int64_t timeout_us,
                    int64_t grace_us) const {
    if (!pipeline_start_us || now_us-pipeline_start_us < grace_us) return "STARTING";
    if (now_us-std::max(pipeline_start_us, last_sdk_us.load()) >= timeout_us)
      return "SDK_STREAM_STALLED";
    const auto since = demand_since_us.load();
    if (since && now_us-std::max(since, last_publish_us.load()) >= timeout_us)
      return "PUBLISH_STALLED";
    return "OK";
  }
};

template<class T> struct QueuePushResult { bool accepted = false; std::optional<T> discarded; };

template<class T> QueuePushResult<T> pushBoundedFrame(std::queue<T> &queue, std::mutex &mutex,
    std::condition_variable &cv, const std::atomic_bool &running, size_t capacity, T value,
    FrameStreamHealth &health, int64_t now_us) {
  QueuePushResult<T> result;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!running) return result;
    if (queue.size() >= capacity) {
      result.discarded = std::move(queue.front()); queue.pop();
      ++health.queue_drop_total; health.last_queue_drop_us = now_us;
    }
    queue.push(std::move(value)); result.accepted = true;
    health.queue_size = queue.size();
    health.queue_high_watermark = std::max(health.queue_high_watermark.load(), queue.size());
  }
  cv.notify_one();
  return result;
}

template<class T> std::optional<T> waitPopFrame(std::queue<T> &queue, std::mutex &mutex,
    std::condition_variable &cv, const std::atomic_bool &running, FrameStreamHealth &health) {
  std::unique_lock<std::mutex> lock(mutex);
  cv.wait(lock, [&]() { return !queue.empty() || !running; });
  if (!running) return std::nullopt;
  auto value = std::move(queue.front()); queue.pop();
  health.queue_size = queue.size();
  return value;
}

// Keeps decoder ownership confined to its single per-stream consumer thread.
// Queue waits/eviction are done by OBCameraNode under its queue mutex only.
template<class F> class ScopeExit {
 public:
  explicit ScopeExit(F action) : action_(std::move(action)) {}
  ~ScopeExit() noexcept { try { action_(); } catch (...) {} }
  ScopeExit(const ScopeExit&) = delete;
 private: F action_;
};
template<class F> ScopeExit<F> onScopeExit(F action) { return ScopeExit<F>(std::move(action)); }

}  // namespace orbbec_camera
