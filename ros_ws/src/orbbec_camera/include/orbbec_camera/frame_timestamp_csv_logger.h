#pragma once

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <map>
#include <tuple>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

#include "libobsensor/ObSensor.hpp"

namespace orbbec_camera {

class FrameTimestampCsvLogger {
 public:
  FrameTimestampCsvLogger(bool enabled, const std::string &csv_file_path, rclcpp::Logger logger,
                          size_t max_pending_rows = 256, int64_t pending_timeout_us = 2000000,
                          size_t max_completed_rows = 1000);

  ~FrameTimestampCsvLogger() noexcept;

  FrameTimestampCsvLogger(const FrameTimestampCsvLogger &) = delete;
  FrameTimestampCsvLogger &operator=(const FrameTimestampCsvLogger &) = delete;

  void recordFrameSet(const std::shared_ptr<ob::Frame> &color_frame,
                      const std::shared_ptr<ob::Frame> &depth_frame, int64_t arrival_system_us,
                      int64_t arrival_steady_us, bool track_color, bool track_depth,
                      bool color_image_publish_expected, bool depth_image_publish_expected);

  void recordStandaloneFrameArrival(OBStreamType stream_type,
                                    const std::shared_ptr<ob::Frame> &frame,
                                    int64_t arrival_system_us, int64_t arrival_steady_us,
                                    bool image_publish_expected);

  void recordPreImagePublish(OBStreamType stream_type, const std::shared_ptr<ob::Frame> &frame,
                             int64_t publish_system_us, int64_t publish_steady_us);

  void cancelFrame(OBStreamType stream_type, const std::shared_ptr<ob::Frame> &frame,
                   const std::string &reason);
  struct Stats { size_t pending, mappings, completed; uint64_t expired, capacity, cancelled, completed_dropped; bool write_failed; };
  Stats stats() const;
  void shutdown();

  bool enabled() const { return enabled_; }

 private:
  enum class TrackedStream { COLOR, DEPTH };
  using FrameKey = std::tuple<uint64_t, uint64_t, uint64_t>;
  static FrameKey frameKey(const std::shared_ptr<ob::Frame> &frame);

  struct StreamState {
    bool has_frame = false;
    bool publish_expected = false;
    bool final = false;
    uint64_t frame_index = 0;
    FrameKey frame_key{};
    std::string completion_reason;

    std::optional<int64_t> metadata_frame_number;
    std::optional<int64_t> sensor_ts_us;
    int64_t device_ts_us = 0;
    int64_t global_ts_us = 0;
    int64_t sdk_system_ts_us = 0;
    int64_t arrival_system_us = 0;
    int64_t arrival_steady_us = 0;
    std::optional<int64_t> publish_system_us;
    std::optional<int64_t> publish_steady_us;

    std::optional<int64_t> device_ts_delta_us;
    std::optional<int64_t> sensor_ts_delta_us;
    std::optional<int64_t> global_ts_delta_us;
    std::optional<int64_t> sdk_system_ts_delta_us;
    std::optional<int64_t> arrival_system_delta_us;
    std::optional<int64_t> arrival_steady_delta_us;
    std::optional<int64_t> publish_system_delta_us;
    std::optional<int64_t> publish_steady_delta_us;
    std::optional<int64_t> arrival_to_publish_system_us;
    std::optional<int64_t> arrival_to_publish_steady_us;
    std::optional<int64_t> sdk_delay_from_global_us;
    std::optional<int64_t> sdk_delay_from_system_us;
  };

  struct PendingRow {
    uint64_t row_id = 0;
    int64_t created_steady_us = 0;
    StreamState color;
    StreamState depth;
  };

  struct PreviousStreamTimestamps {
    std::optional<int64_t> device_ts_us;
    std::optional<int64_t> sensor_ts_us;
    std::optional<int64_t> global_ts_us;
    std::optional<int64_t> sdk_system_ts_us;
    std::optional<int64_t> arrival_system_us;
    std::optional<int64_t> arrival_steady_us;
    std::optional<int64_t> publish_system_us;
    std::optional<int64_t> publish_steady_us;
  };

  TrackedStream toTrackedStream(OBStreamType stream_type) const;
  bool isTrackedStream(OBStreamType stream_type) const;

  void recordFrameSetInternal(const std::shared_ptr<ob::Frame> &color_frame,
                              const std::shared_ptr<ob::Frame> &depth_frame,
                              int64_t arrival_system_us, int64_t arrival_steady_us,
                              bool track_color, bool track_depth, bool color_image_publish_expected,
                              bool depth_image_publish_expected);
  void recordStandaloneFrameArrivalInternal(OBStreamType stream_type,
                                            const std::shared_ptr<ob::Frame> &frame,
                                            int64_t arrival_system_us, int64_t arrival_steady_us,
                                            bool image_publish_expected);
  void recordPreImagePublishInternal(OBStreamType stream_type,
                                     const std::shared_ptr<ob::Frame> &frame,
                                     int64_t publish_system_us, int64_t publish_steady_us);

  void populateArrivalData(StreamState &state, TrackedStream stream,
                           const std::shared_ptr<ob::Frame> &frame, int64_t arrival_system_us,
                           int64_t arrival_steady_us, bool publish_expected);
  void populatePublishData(StreamState &state, TrackedStream stream, int64_t publish_system_us,
                           int64_t publish_steady_us);

  std::optional<int64_t> updateDelta(std::optional<int64_t> &previous, int64_t current);

  void finalizeStreamWithoutPublish(StreamState &state);
  bool isRowReady(const PendingRow &row) const;
  void enqueueCompletedRow(const PendingRow &row);
  void flushPendingRowsLocked(std::vector<PendingRow> &rows);
  void eraseFrameIndexMappingLocked(const PendingRow &row);

  std::string serializeRow(const PendingRow &row) const;
  std::string serializeStreamColumns(const StreamState &state) const;
  static std::string formatSecondsColumn(int64_t time_us);
  static std::string formatOptionalIntColumn(const std::optional<int64_t> &value);
  static std::string csvHeader();

  void writerThreadMain();
  void openCsvIfNeeded();

  rclcpp::Logger logger_;
  bool enabled_ = false;
  std::atomic_bool shutdown_requested_{false};
  std::atomic_bool writer_failed_{false};
  size_t max_pending_rows_, max_completed_rows_;
  int64_t pending_timeout_us_;
  std::atomic<uint64_t> expired_total_{0}, capacity_total_{0}, cancelled_total_{0}, completed_drop_total_{0};
  void finishRowLocked(uint64_t row_id, const std::string &reason);
  void prepareFrameLocked(const std::shared_ptr<ob::Frame> &frame, TrackedStream stream);
  void expirePending(int64_t now_us);
  void limitPendingLocked();
  std::atomic_bool writer_draining_{false};
  std::string csv_file_path_;
  std::ofstream csv_stream_;
  std::thread writer_thread_;

  uint64_t next_row_id_ = 1;
  std::unordered_map<uint64_t, PendingRow> pending_rows_;
  std::map<FrameKey, uint64_t> color_frame_index_to_row_id_;
  std::map<FrameKey, uint64_t> depth_frame_index_to_row_id_;
  PreviousStreamTimestamps color_previous_;
  PreviousStreamTimestamps depth_previous_;

  std::deque<PendingRow> completed_rows_;

  mutable std::mutex state_mutex_;
  mutable std::mutex completed_rows_mutex_;
  std::condition_variable completed_rows_cv_;
};

}  // namespace orbbec_camera
