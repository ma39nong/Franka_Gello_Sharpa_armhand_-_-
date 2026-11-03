// Synthetic frames only: never creates an SDK Device, Pipeline or Context.
#include "orbbec_camera/frame_stream_health.h"
#include "orbbec_camera/frame_timestamp_csv_logger.h"
#include <chrono>
#include <filesystem>
#include <future>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <thread>

using namespace orbbec_camera;
using namespace std::chrono_literals;
static void check(bool value, const char *what) { if (!value) throw std::runtime_error(what); }
static int64_t nowUs() { return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count(); }
static auto frame() {
  static std::atomic<uint64_t> timestamp{1};
  auto value = ob::FrameFactory::createVideoFrame(OB_FRAME_COLOR, OB_FORMAT_RGB, 2, 2);
  ob::FrameHelper::setFrameDeviceTimestampUs(value, timestamp++);
  return value;
}

static void watchdogTest() {
  const int64_t start = 1000000;
  FrameStreamHealth color, depth;
  check(color.state(start+999999, start, 1000000, 3000000)=="STARTING", "startup grace");
  check(color.state(start+3000000, start, 1000000, 3000000)=="SDK_STREAM_STALLED", "absent SDK frames");
  color.arrival(start+3900000, 10, 240000);
  check(depth.state(start+4000000, start, 1000000, 3000000)=="SDK_STREAM_STALLED", "streams independent");
  check(color.state(start+4000000, start, 1000000, 3000000)=="OK", "no subscriber is not publish stall");
  color.demand(true, start+2000000);
  check(color.state(start+4000000, start, 1000000, 3000000)=="PUBLISH_STALLED", "SDK alive, publish stalled");
  color.published(start+4000000, 10, 240000);
  check(color.state(start+4000000, start, 1000000, 3000000)=="OK", "publish recovery");
  color.demand(false, start+4000000);
  color.arrival(start+5100000, 11, 240000);
  check(color.state(start+5200000, start, 1000000, 3000000)=="OK", "unsubscribe suppresses publish stall");
  check(color.sdk_gap_total==1, "240ms diagnostic gap count");
  color.published(start+5200000, 11, 240000);
  check(color.publish_gap_total==0, "intentional unsubscribe is not a publish quality gap");
}

static void queueTest() {
  std::queue<int> queue;
  std::mutex mutex; std::condition_variable cv; std::atomic_bool running{true};
  FrameStreamHealth health;
  pushBoundedFrame(queue, mutex, cv, running, 4, -1, health, nowUs());
  std::promise<void> decoding, release; auto released = release.get_future();
  auto consumer = std::async(std::launch::async, [&]() {
    auto item = waitPopFrame(queue, mutex, cv, running, health);
    check(item && *item==-1, "first frame retained");
    decoding.set_value(); released.wait(); // Slow decode must not retain queue mutex.
    return waitPopFrame(queue, mutex, cv, running, health);
  });
  decoding.get_future().wait();
  const auto begin = nowUs();
  auto producer = std::async(std::launch::async, [&]() {
    for (int i=0; i<10000; ++i) pushBoundedFrame(queue, mutex, cv, running, 4, i, health, nowUs());
  });
  const bool finished = producer.wait_for(500ms)==std::future_status::ready;
  running=false; cv.notify_all(); release.set_value(); producer.get();
  check(finished, "producer blocked behind decoder");
  check(!consumer.get(), "consumer stops with queued frames");
  check(health.queue_high_watermark==4 && queue.size()==4, "bounded queue");
  check(health.queue_drop_total==9996 && queue.front()==9996, "old frames evicted and counted");
  std::cout << "queue: 10000 submissions us=" << nowUs()-begin << " drops=" << health.queue_drop_total << '\n';
  std::queue<int> empty;
  running=true;
  auto waiter=std::async(std::launch::async, [&]() { return waitPopFrame(empty, mutex, cv, running, health); });
  running=false; cv.notify_all();
  check(waiter.wait_for(500ms)==std::future_status::ready && !waiter.get(), "empty waiter wakes on stop");
}

static void csvTest(const std::filesystem::path &dir) {
  auto log = rclcpp::get_logger("offline_camera_csv");
  { FrameTimestampCsvLogger disabled(false, (dir/"disabled.csv").string(), log);
    disabled.recordFrameSet(frame(), nullptr, nowUs(), nowUs(), true, false, true, false);
    check(disabled.stats().pending==0, "disabled logger allocates no pending rows"); }
  check(!std::filesystem::exists(dir/"disabled.csv"), "disabled creates no file");
  { FrameTimestampCsvLogger failed(true, "/dev/full", log);
    check(!failed.enabled() && failed.stats().write_failed, "write failure is explicit"); }
  for (int camera=0; camera<3; ++camera) {
    auto path=dir/("cam"+std::to_string(camera)+".csv");
    FrameTimestampCsvLogger csv(true, path.string(), log, 4, 20000, 8);
    auto a=frame(), b=frame(); // Factory frames reuse SDK index 0: must not alias.
    auto frames = ob::FrameFactory::createFrameSet();
    frames->pushFrame(a);
    auto arrival_wrapper = frames->getFrame(OB_FRAME_COLOR);
    auto consumer_wrapper = frames->getFrame(OB_FRAME_COLOR);
    check(arrival_wrapper.get()!=consumer_wrapper.get(), "SDK returns distinct wrappers");
    csv.recordFrameSet(arrival_wrapper, nullptr, nowUs(), nowUs(), true, false, true, false);
    csv.recordPreImagePublish(OB_STREAM_COLOR, consumer_wrapper, nowUs(), nowUs());
    check(csv.stats().pending==0, "different wrappers for same SDK frame correlate");
    csv.recordFrameSet(a, nullptr, nowUs(), nowUs(), true, false, true, false);
    csv.recordFrameSet(b, nullptr, nowUs(), nowUs(), true, false, true, false);
    check(csv.stats().pending==2, "same SDK index retains separate frame identities");
    csv.recordPreImagePublish(OB_STREAM_COLOR, a, nowUs(), nowUs());
    check(csv.stats().pending==1, "publish completes exact frame");
    csv.cancelFrame(OB_STREAM_COLOR, b, "queue_overflow");
    check(csv.stats().pending==0 && csv.stats().mappings==0, "cancel removes mapping");
    csv.recordStandaloneFrameArrival(OB_STREAM_COLOR, a, nowUs(), nowUs(), true);
    csv.cancelFrame(OB_STREAM_COLOR, a, "subscriber_left");
    check(csv.stats().pending==0, "standalone cancellation");
    csv.recordFrameSet(a, nullptr, nowUs(), nowUs(), true, false, false, false);
    check(csv.stats().pending==0, "no subscriber completes immediately");
    csv.recordFrameSet(a, nullptr, nowUs(), nowUs(), true, false, true, false);
    csv.recordFrameSet(a, nullptr, nowUs(), nowUs(), true, false, true, false);
    check(csv.stats().pending==1, "same identity re-arrival finalizes old row");
    csv.cancelFrame(OB_STREAM_COLOR, a, "publish_failed");
    std::vector<std::shared_ptr<ob::Frame>> held;
    for (int i=0; i<100; ++i) {
      held.push_back(frame());
      csv.recordFrameSet(held.back(), nullptr, nowUs(), nowUs(), true, false, true, false);
      check(csv.stats().pending<=4 && csv.stats().mappings<=4 && csv.stats().completed<=8, "all queues bounded");
    }
    check(csv.stats().capacity>0, "capacity loss recorded");
    std::this_thread::sleep_for(1100ms);
    check(csv.stats().pending==0 && csv.stats().expired>0, "pending expires without new arrivals");
    // Both subscribers leave, then rejoin; scopes/queue cancellation use same API.
    csv.recordFrameSet(a, b, nowUs(), nowUs(), true, true, true, true);
    csv.cancelFrame(OB_STREAM_COLOR, a, "no_subscriber");
    check(csv.stats().pending==1, "paired row waits for remaining stream");
    csv.recordPreImagePublish(OB_STREAM_DEPTH, b, nowUs(), nowUs());
    check(csv.stats().pending==0 && csv.stats().mappings==0, "paired cancellation completes correctly");
    csv.recordFrameSet(a, nullptr, nowUs(), nowUs(), true, false, true, false);
    csv.shutdown(); csv.shutdown();
    check(csv.stats().pending==0 && csv.stats().mappings==0, "idempotent shutdown clears pending");
    std::ifstream f(path); std::string line; std::getline(f, line);
    check(line.find("color_completion_reason")!=std::string::npos, "completion reason persisted");
    std::cout << "csv camera=" << camera << " expired=" << csv.stats().expired << " capacity=" << csv.stats().capacity << '\n';
  }
  // A fresh logger after reconnect cannot correlate against a previous run.
  { FrameTimestampCsvLogger csv(true, (dir/"restart.csv").string(), log);
    check(csv.stats().mappings==0, "restart has clean identity map"); }
}

int main() {
  const auto dir=std::filesystem::temp_directory_path()/("camera-offline-"+std::to_string(nowUs()));
  std::filesystem::create_directories(dir);
  try { watchdogTest(); queueTest(); csvTest(dir); }
  catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
  std::filesystem::remove_all(dir);
  std::cout << "camera driver offline tests passed\n";
}
