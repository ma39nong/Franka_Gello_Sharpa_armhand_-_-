#include "orbbec_camera/frame_timestamp_csv_logger.h"

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>

namespace orbbec_camera {
namespace {

int64_t steadyNowUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
constexpr size_t kFlushBatchSize = 100;
constexpr auto kFlushInterval = std::chrono::seconds(1);

}  // namespace

FrameTimestampCsvLogger::FrameTimestampCsvLogger(bool enabled, const std::string &csv_file_path,
                                                 rclcpp::Logger logger, size_t max_pending_rows,
                                                 int64_t pending_timeout_us, size_t max_completed_rows)
    : logger_(std::move(logger)), enabled_(enabled),
      max_pending_rows_(std::max<size_t>(1, max_pending_rows)),
      max_completed_rows_(std::max<size_t>(1, max_completed_rows)),
      pending_timeout_us_(std::max<int64_t>(1, pending_timeout_us)), csv_file_path_(csv_file_path) {
  if (!enabled_) {
    return;
  }

  try {
    auto path = std::filesystem::path(csv_file_path_);
    if (path.has_parent_path() && !std::filesystem::exists(path.parent_path())) {
      std::filesystem::create_directories(path.parent_path());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to prepare frame timestamp CSV path " << csv_file_path_
                                                                               << ": " << e.what());
    enabled_ = false;
    writer_failed_ = true;
    return;
  }

  openCsvIfNeeded();
  if (!enabled_) {
    return;
  }

  writer_thread_ = std::thread([this]() { writerThreadMain(); });
}

FrameTimestampCsvLogger::~FrameTimestampCsvLogger() noexcept { shutdown(); }

void FrameTimestampCsvLogger::recordFrameSet(const std::shared_ptr<ob::Frame> &color_frame,
                                             const std::shared_ptr<ob::Frame> &depth_frame,
                                             int64_t arrival_system_us, int64_t arrival_steady_us,
                                             bool track_color, bool track_depth,
                                             bool color_image_publish_expected,
                                             bool depth_image_publish_expected) {
  if (!enabled_ || writer_failed_) {
    return;
  }
  recordFrameSetInternal(color_frame, depth_frame, arrival_system_us, arrival_steady_us,
                         track_color, track_depth, color_image_publish_expected,
                         depth_image_publish_expected);
}

void FrameTimestampCsvLogger::recordStandaloneFrameArrival(OBStreamType stream_type,
                                                           const std::shared_ptr<ob::Frame> &frame,
                                                           int64_t arrival_system_us,
                                                           int64_t arrival_steady_us,
                                                           bool image_publish_expected) {
  if (!enabled_ || writer_failed_ || !frame || !isTrackedStream(stream_type)) {
    return;
  }
  recordStandaloneFrameArrivalInternal(stream_type, frame, arrival_system_us, arrival_steady_us,
                                       image_publish_expected);
}

void FrameTimestampCsvLogger::recordPreImagePublish(OBStreamType stream_type,
                                                    const std::shared_ptr<ob::Frame> &frame,
                                                    int64_t publish_system_us,
                                                    int64_t publish_steady_us) {
  if (!enabled_ || writer_failed_ || !frame || !isTrackedStream(stream_type)) {
    return;
  }
  recordPreImagePublishInternal(stream_type, frame, publish_system_us, publish_steady_us);
}

void FrameTimestampCsvLogger::shutdown() {
  if (!enabled_) {
    return;
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (shutdown_requested_) {
      return;
    }
    shutdown_requested_ = true;

    std::vector<PendingRow> rows_to_flush;
    flushPendingRowsLocked(rows_to_flush);
    for (const auto &row : rows_to_flush) {
      enqueueCompletedRow(row);
    }
  }

  writer_draining_ = true;
  completed_rows_cv_.notify_all();
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  if (csv_stream_.is_open()) {
    csv_stream_.flush();
    csv_stream_.close();
  }
}

FrameTimestampCsvLogger::TrackedStream FrameTimestampCsvLogger::toTrackedStream(
    OBStreamType stream_type) const {
  if (stream_type == OB_STREAM_COLOR) {
    return TrackedStream::COLOR;
  }
  return TrackedStream::DEPTH;
}

bool FrameTimestampCsvLogger::isTrackedStream(OBStreamType stream_type) const {
  return stream_type == OB_STREAM_COLOR || stream_type == OB_STREAM_DEPTH;
}

void FrameTimestampCsvLogger::recordFrameSetInternal(
    const std::shared_ptr<ob::Frame> &color_frame, const std::shared_ptr<ob::Frame> &depth_frame,
    int64_t arrival_system_us, int64_t arrival_steady_us, bool track_color, bool track_depth,
    bool color_image_publish_expected, bool depth_image_publish_expected) {
  if (!track_color && !track_depth) {
    return;
  }

  std::vector<PendingRow> ready_rows;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (shutdown_requested_) {
      return;
    }

    PendingRow row;
    row.row_id = next_row_id_++;
    row.created_steady_us = steadyNowUs();

    if (track_color && color_frame) {
      populateArrivalData(row.color, TrackedStream::COLOR, color_frame, arrival_system_us,
                          arrival_steady_us, color_image_publish_expected);
      prepareFrameLocked(color_frame, TrackedStream::COLOR);
      color_frame_index_to_row_id_[row.color.frame_key] = row.row_id;
      if (!color_image_publish_expected) {
        finalizeStreamWithoutPublish(row.color);
      }
    } else {
      row.color.final = true;
    }

    if (track_depth && depth_frame) {
      populateArrivalData(row.depth, TrackedStream::DEPTH, depth_frame, arrival_system_us,
                          arrival_steady_us, depth_image_publish_expected);
      prepareFrameLocked(depth_frame, TrackedStream::DEPTH);
      depth_frame_index_to_row_id_[row.depth.frame_key] = row.row_id;
      if (!depth_image_publish_expected) {
        finalizeStreamWithoutPublish(row.depth);
      }
    } else {
      row.depth.final = true;
    }

    pending_rows_.emplace(row.row_id, row);
    limitPendingLocked();
    if (isRowReady(row)) {
      auto it = pending_rows_.find(row.row_id);
      if (it != pending_rows_.end()) {
        ready_rows.push_back(it->second);
        eraseFrameIndexMappingLocked(it->second);
        pending_rows_.erase(it);
      }
    }
  }

  for (const auto &ready_row : ready_rows) {
    enqueueCompletedRow(ready_row);
  }
}

void FrameTimestampCsvLogger::recordStandaloneFrameArrivalInternal(
    OBStreamType stream_type, const std::shared_ptr<ob::Frame> &frame, int64_t arrival_system_us,
    int64_t arrival_steady_us, bool image_publish_expected) {
  std::optional<PendingRow> ready_row;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (shutdown_requested_) {
      return;
    }

    PendingRow row;
    row.row_id = next_row_id_++;
    row.created_steady_us = steadyNowUs();

    const auto tracked_stream = toTrackedStream(stream_type);
    auto &state = tracked_stream == TrackedStream::COLOR ? row.color : row.depth;
    auto &other_state = tracked_stream == TrackedStream::COLOR ? row.depth : row.color;

    populateArrivalData(state, tracked_stream, frame, arrival_system_us, arrival_steady_us,
                        image_publish_expected);
    other_state.final = true;

    prepareFrameLocked(frame, tracked_stream);
    if (tracked_stream == TrackedStream::COLOR) {
      color_frame_index_to_row_id_[state.frame_key] = row.row_id;
    } else {
      depth_frame_index_to_row_id_[state.frame_key] = row.row_id;
    }

    if (!image_publish_expected) {
      finalizeStreamWithoutPublish(state);
    }

    pending_rows_.emplace(row.row_id, row);
    limitPendingLocked();
    if (isRowReady(row)) {
      auto it = pending_rows_.find(row.row_id);
      if (it != pending_rows_.end()) {
        ready_row = it->second;
        eraseFrameIndexMappingLocked(*ready_row);
        pending_rows_.erase(it);
      }
    }
  }

  if (ready_row.has_value()) {
    enqueueCompletedRow(*ready_row);
  }
}

void FrameTimestampCsvLogger::recordPreImagePublishInternal(OBStreamType stream_type,
                                                            const std::shared_ptr<ob::Frame> &frame,
                                                            int64_t publish_system_us,
                                                            int64_t publish_steady_us) {
  std::optional<PendingRow> ready_row;
  const auto frame_key = frameKey(frame);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (shutdown_requested_) {
      return;
    }

    const auto tracked_stream = toTrackedStream(stream_type);
    auto &row_map = tracked_stream == TrackedStream::COLOR ? color_frame_index_to_row_id_
                                                           : depth_frame_index_to_row_id_;
    auto row_id_it = row_map.find(frame_key);
    if (row_id_it == row_map.end()) {
      // No publish was expected, or this exact frame was cancelled/expired.
      return;
    }
    const auto row_id = row_id_it->second;

    auto pending_it = pending_rows_.find(row_id);
    if (pending_it == pending_rows_.end()) {
      return;
    }

    auto &state = tracked_stream == TrackedStream::COLOR ? pending_it->second.color
                                                         : pending_it->second.depth;
    if (state.final) return;
    populatePublishData(state, tracked_stream, publish_system_us, publish_steady_us);
    state.final = true;
    state.completion_reason = "published";

    if (isRowReady(pending_it->second)) {
      ready_row = pending_it->second;
      eraseFrameIndexMappingLocked(*ready_row);
      pending_rows_.erase(pending_it);
    }
  }

  if (ready_row.has_value()) {
    enqueueCompletedRow(*ready_row);
  }
}

void FrameTimestampCsvLogger::populateArrivalData(StreamState &state, TrackedStream stream,
                                                  const std::shared_ptr<ob::Frame> &frame,
                                                  int64_t arrival_system_us,
                                                  int64_t arrival_steady_us,
                                                  bool publish_expected) {
  auto &previous = stream == TrackedStream::COLOR ? color_previous_ : depth_previous_;

  state.has_frame = true;
  state.publish_expected = publish_expected;
  state.frame_index = frame->getIndex();
  state.frame_key = frameKey(frame);
  if (frame->hasMetadata(OB_FRAME_METADATA_TYPE_FRAME_NUMBER)) {
    state.metadata_frame_number =
        static_cast<int64_t>(frame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER));
  } else {
    state.metadata_frame_number.reset();
  }
  if (frame->hasMetadata(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP)) {
    state.sensor_ts_us =
        static_cast<int64_t>(frame->getMetadataValue(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP));
  } else {
    state.sensor_ts_us.reset();
  }
  state.device_ts_us = static_cast<int64_t>(frame->getTimeStampUs());
  state.global_ts_us = static_cast<int64_t>(frame->getGlobalTimeStampUs());
  state.sdk_system_ts_us = static_cast<int64_t>(frame->getSystemTimeStampUs());
  state.arrival_system_us = arrival_system_us;
  state.arrival_steady_us = arrival_steady_us;
  state.device_ts_delta_us = updateDelta(previous.device_ts_us, state.device_ts_us);
  if (state.sensor_ts_us.has_value()) {
    state.sensor_ts_delta_us = updateDelta(previous.sensor_ts_us, state.sensor_ts_us.value());
  } else {
    state.sensor_ts_delta_us.reset();
    previous.sensor_ts_us.reset();
  }
  state.global_ts_delta_us = updateDelta(previous.global_ts_us, state.global_ts_us);
  state.sdk_system_ts_delta_us = updateDelta(previous.sdk_system_ts_us, state.sdk_system_ts_us);
  state.arrival_system_delta_us = updateDelta(previous.arrival_system_us, state.arrival_system_us);
  state.arrival_steady_delta_us = updateDelta(previous.arrival_steady_us, state.arrival_steady_us);
  state.sdk_delay_from_global_us = state.arrival_system_us - state.global_ts_us;
  state.sdk_delay_from_system_us = state.arrival_system_us - state.sdk_system_ts_us;
}

void FrameTimestampCsvLogger::populatePublishData(StreamState &state, TrackedStream stream,
                                                  int64_t publish_system_us,
                                                  int64_t publish_steady_us) {
  auto &previous = stream == TrackedStream::COLOR ? color_previous_ : depth_previous_;

  state.publish_system_us = publish_system_us;
  state.publish_steady_us = publish_steady_us;
  state.publish_system_delta_us =
      updateDelta(previous.publish_system_us, state.publish_system_us.value());
  state.publish_steady_delta_us =
      updateDelta(previous.publish_steady_us, state.publish_steady_us.value());
  state.arrival_to_publish_system_us = state.publish_system_us.value() - state.arrival_system_us;
  state.arrival_to_publish_steady_us = state.publish_steady_us.value() - state.arrival_steady_us;
}

std::optional<int64_t> FrameTimestampCsvLogger::updateDelta(std::optional<int64_t> &previous,
                                                            int64_t current) {
  std::optional<int64_t> delta;
  if (previous.has_value()) {
    delta = current - previous.value();
  }
  previous = current;
  return delta;
}

void FrameTimestampCsvLogger::finalizeStreamWithoutPublish(StreamState &state) {
  state.final = true;
  state.completion_reason = "no_raw_subscriber";
}

bool FrameTimestampCsvLogger::isRowReady(const PendingRow &row) const {
  return row.color.final && row.depth.final;
}

void FrameTimestampCsvLogger::enqueueCompletedRow(const PendingRow &row) {
  std::lock_guard<std::mutex> queue_lock(completed_rows_mutex_);
  if (completed_rows_.size() >= max_completed_rows_) {
    completed_rows_.pop_front();
    const auto count = ++completed_drop_total_;
    if ((count & (count - 1)) == 0)
      RCLCPP_ERROR_STREAM(logger_, "CSV completed queue full; discarded rows total=" << count);
  }
  completed_rows_.push_back(row);
  completed_rows_cv_.notify_one();
}

void FrameTimestampCsvLogger::flushPendingRowsLocked(std::vector<PendingRow> &rows) {
  rows.reserve(rows.size() + pending_rows_.size());
  for (auto &item : pending_rows_) {
    auto row = item.second;
    for (auto *state : {&row.color, &row.depth}) {
      if (!state->final) { state->final = true; state->completion_reason = "shutdown"; }
    }
    rows.push_back(std::move(row));
  }
  pending_rows_.clear();
  color_frame_index_to_row_id_.clear();
  depth_frame_index_to_row_id_.clear();
}

void FrameTimestampCsvLogger::eraseFrameIndexMappingLocked(const PendingRow &row) {
  for (auto stream : {TrackedStream::COLOR, TrackedStream::DEPTH}) {
    const auto &state = stream == TrackedStream::COLOR ? row.color : row.depth;
    auto &map = stream == TrackedStream::COLOR ? color_frame_index_to_row_id_ : depth_frame_index_to_row_id_;
    auto it = map.find(state.frame_key);
    if (it != map.end() && it->second == row.row_id) map.erase(it);
  }
}

std::string FrameTimestampCsvLogger::serializeRow(const PendingRow &row) const {
  std::ostringstream ss;
  ss << serializeStreamColumns(row.color) << "," << serializeStreamColumns(row.depth)
     << "," << row.color.completion_reason << "," << row.depth.completion_reason;
  return ss.str();
}

std::string FrameTimestampCsvLogger::serializeStreamColumns(const StreamState &state) const {
  std::vector<std::string> fields(22, "");
  if (state.has_frame) {
    fields[0] = std::to_string(state.frame_index);
    fields[1] = formatOptionalIntColumn(state.metadata_frame_number);
    if (state.sensor_ts_us.has_value()) {
      fields[2] = formatSecondsColumn(state.sensor_ts_us.value());
    }
    fields[3] = formatOptionalIntColumn(state.sensor_ts_delta_us);
    fields[4] = formatSecondsColumn(state.device_ts_us);
    fields[5] = formatOptionalIntColumn(state.device_ts_delta_us);
    fields[6] = formatSecondsColumn(state.global_ts_us);
    fields[7] = formatOptionalIntColumn(state.global_ts_delta_us);
    fields[8] = formatSecondsColumn(state.sdk_system_ts_us);
    fields[9] = formatOptionalIntColumn(state.sdk_system_ts_delta_us);
    fields[10] = formatSecondsColumn(state.arrival_system_us);
    fields[11] = formatOptionalIntColumn(state.arrival_system_delta_us);
    fields[12] = formatSecondsColumn(state.arrival_steady_us);
    fields[13] = formatOptionalIntColumn(state.arrival_steady_delta_us);
    if (state.publish_system_us.has_value()) {
      fields[14] = formatSecondsColumn(state.publish_system_us.value());
    }
    fields[15] = formatOptionalIntColumn(state.publish_system_delta_us);
    if (state.publish_steady_us.has_value()) {
      fields[16] = formatSecondsColumn(state.publish_steady_us.value());
    }
    fields[17] = formatOptionalIntColumn(state.publish_steady_delta_us);
    fields[18] = formatOptionalIntColumn(state.arrival_to_publish_system_us);
    fields[19] = formatOptionalIntColumn(state.arrival_to_publish_steady_us);
    fields[20] = formatOptionalIntColumn(state.sdk_delay_from_global_us);
    fields[21] = formatOptionalIntColumn(state.sdk_delay_from_system_us);
  }

  std::ostringstream ss;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i != 0) {
      ss << ",";
    }
    ss << fields[i];
  }
  return ss.str();
}

std::string FrameTimestampCsvLogger::formatSecondsColumn(int64_t time_us) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << (static_cast<long double>(time_us) / 1000000.0L);
  return ss.str();
}

std::string FrameTimestampCsvLogger::formatOptionalIntColumn(const std::optional<int64_t> &value) {
  if (!value.has_value()) {
    return "";
  }
  return std::to_string(*value);
}

std::string FrameTimestampCsvLogger::csvHeader() {
  std::ostringstream ss;
  for (const auto *prefix : {"color", "depth"}) {
    ss << prefix << "_sdk_frame_index,";
    ss << prefix << "_hardware_frame_number,";
    ss << prefix << "_sensor_ts_sec,";
    ss << prefix << "_sensor_ts_delta_us,";
    ss << prefix << "_device_ts_sec,";
    ss << prefix << "_device_ts_delta_us,";
    ss << prefix << "_global_ts_sec,";
    ss << prefix << "_global_ts_delta_us,";
    ss << prefix << "_system_ts_sec,";
    ss << prefix << "_system_ts_delta_us,";
    ss << prefix << "_arrival_system_sec,";
    ss << prefix << "_arrival_system_delta_us,";
    ss << prefix << "_arrival_steady_sec,";
    ss << prefix << "_arrival_steady_delta_us,";
    ss << prefix << "_publish_system_sec,";
    ss << prefix << "_publish_system_delta_us,";
    ss << prefix << "_publish_steady_sec,";
    ss << prefix << "_publish_steady_delta_us,";
    ss << prefix << "_arrival_to_publish_system_us,";
    ss << prefix << "_arrival_to_publish_steady_us,";
    ss << prefix << "_sdk_delay_from_global_us,";
    ss << prefix << "_sdk_delay_from_system_us";
    if (std::string(prefix) == "color") {
      ss << ",";
    }
  }
  return ss.str() + ",color_completion_reason,depth_completion_reason";
}

void FrameTimestampCsvLogger::finishRowLocked(uint64_t row_id, const std::string &reason) {
  auto it = pending_rows_.find(row_id);
  if (it == pending_rows_.end()) return;
  for (auto *state : {&it->second.color, &it->second.depth}) {
    if (!state->final) { state->final = true; state->completion_reason = reason; }
  }
  eraseFrameIndexMappingLocked(it->second);
  enqueueCompletedRow(it->second);
  pending_rows_.erase(it);
}

FrameTimestampCsvLogger::FrameKey FrameTimestampCsvLogger::frameKey(
    const std::shared_ptr<ob::Frame> &frame) {
  // FrameSet getters create fresh C++/C wrappers. Addresses are NOT frame
  // identity. The captured tuple survives those wrappers, including reuse of
  // an SDK frame index after reconnect; an exact duplicate retires the old row.
  return {frame->getIndex(), frame->getTimeStampUs(), frame->getSystemTimeStampUs()};
}

void FrameTimestampCsvLogger::prepareFrameLocked(const std::shared_ptr<ob::Frame> &frame,
                                                TrackedStream stream) {
  auto &map = stream == TrackedStream::COLOR ? color_frame_index_to_row_id_ : depth_frame_index_to_row_id_;
  auto it = map.find(frameKey(frame));
  if (it != map.end()) {
    ++cancelled_total_;
    finishRowLocked(it->second, "frame_identity_reused");
  }
}

void FrameTimestampCsvLogger::limitPendingLocked() {
  while (pending_rows_.size() > max_pending_rows_) {
    auto oldest = std::min_element(pending_rows_.begin(), pending_rows_.end(),
        [](const auto &a, const auto &b) { return a.first < b.first; });
    const auto count = ++capacity_total_;
    if ((count & (count - 1)) == 0)
      RCLCPP_ERROR_STREAM(logger_, "CSV pending limit exceeded; finalized rows total=" << count);
    finishRowLocked(oldest->first, "pending_capacity");
  }
}

void FrameTimestampCsvLogger::expirePending(int64_t now_us) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<uint64_t> expired;
  for (const auto &entry : pending_rows_)
    if (now_us - entry.second.created_steady_us >= pending_timeout_us_) expired.push_back(entry.first);
  for (auto id : expired) { ++expired_total_; finishRowLocked(id, "publish_timeout"); }
  if (!expired.empty()) RCLCPP_WARN_STREAM(logger_, "CSV publish timeout; expired total=" << expired_total_);
}

void FrameTimestampCsvLogger::cancelFrame(OBStreamType stream_type,
    const std::shared_ptr<ob::Frame> &frame, const std::string &reason) {
  if (!enabled_ || !frame || !isTrackedStream(stream_type)) return;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (shutdown_requested_) return;
  auto &map = stream_type == OB_STREAM_COLOR ? color_frame_index_to_row_id_ : depth_frame_index_to_row_id_;
  auto it = map.find(frameKey(frame));
  if (it == map.end()) return;
  auto row = pending_rows_.find(it->second);
  if (row == pending_rows_.end()) return;
  auto &state = stream_type == OB_STREAM_COLOR ? row->second.color : row->second.depth;
  if (state.final) return;
  state.final = true; state.completion_reason = reason; ++cancelled_total_;
  if (isRowReady(row->second)) finishRowLocked(row->first, reason);
}

FrameTimestampCsvLogger::Stats FrameTimestampCsvLogger::stats() const {
  std::lock_guard<std::mutex> state(state_mutex_);
  std::lock_guard<std::mutex> queue(completed_rows_mutex_);
  return {pending_rows_.size(), color_frame_index_to_row_id_.size()+depth_frame_index_to_row_id_.size(),
          completed_rows_.size(), expired_total_.load(), capacity_total_.load(),
          cancelled_total_.load(), completed_drop_total_.load(), writer_failed_.load()};
}

void FrameTimestampCsvLogger::writerThreadMain() {
  if (!enabled_ || writer_failed_) {
    return;
  }

  size_t rows_since_flush = 0;
  auto last_flush = std::chrono::steady_clock::now();

  while (true) {
    expirePending(steadyNowUs());
    std::deque<PendingRow> rows_to_write;
    {
      std::unique_lock<std::mutex> lock(completed_rows_mutex_);
      completed_rows_cv_.wait_for(lock, kFlushInterval, [this]() {
        return writer_draining_ || !completed_rows_.empty();
      });
      rows_to_write.swap(completed_rows_);
    }

    for (const auto &row : rows_to_write) {
      if (csv_stream_.is_open()) {
        csv_stream_ << serializeRow(row) << "\n";
        rows_since_flush++;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (csv_stream_.is_open() && (rows_since_flush >= kFlushBatchSize ||
                                  now - last_flush >= kFlushInterval || shutdown_requested_)) {
      csv_stream_.flush();
      if (!csv_stream_.good() && !writer_failed_.exchange(true)) {
        RCLCPP_ERROR(logger_, "CSV write failed; diagnostics are incomplete");
      }
      rows_since_flush = 0;
      last_flush = now;
    }

    std::lock_guard<std::mutex> lock(completed_rows_mutex_);
    if (writer_draining_ && completed_rows_.empty()) {
      break;
    }
  }
}

void FrameTimestampCsvLogger::openCsvIfNeeded() {
  csv_stream_.open(csv_file_path_, std::ios::out | std::ios::trunc);
  if (!csv_stream_.is_open()) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to open frame timestamp CSV file: " << csv_file_path_);
    enabled_ = false;
    writer_failed_ = true;
    return;
  }
  csv_stream_ << csvHeader() << "\n";
  csv_stream_.flush();
  if (!csv_stream_.good()) {
    enabled_ = false; writer_failed_ = true;
    RCLCPP_ERROR(logger_, "Failed to write CSV header");
    csv_stream_.close();
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "Frame timestamp CSV logger enabled: " << csv_file_path_);
}

}  // namespace orbbec_camera
