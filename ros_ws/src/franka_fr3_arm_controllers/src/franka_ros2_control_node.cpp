// Copyright 2020 ROS2-Control Development Team
// Copyright 2026 Franka upper-body teleoperation contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace
{
constexpr int kDefaultSchedPriority = 50;
constexpr long kSignalPollNanoseconds = 100'000'000;

bool block_shutdown_signals(sigset_t & shutdown_signals)
{
  sigemptyset(&shutdown_signals);
  sigaddset(&shutdown_signals, SIGINT);
  sigaddset(&shutdown_signals, SIGTERM);
  return pthread_sigmask(SIG_BLOCK, &shutdown_signals, nullptr) == 0;
}
}  // namespace

// This is based on ros2_control 2.54.0's ros2_control_node.  The upstream
// executable lets rclcpp run ControllerManager's pre-shutdown callback while
// its realtime read/update/write thread can still be inside the hardware
// interface.  In an FCI-driven loop that race can leave shutdown waiting
// forever.  Signals are handled synchronously here so the realtime thread is
// joined before rclcpp::shutdown() deactivates controllers and hardware.
int main(int argc, char ** argv)
{
  sigset_t shutdown_signals;
  if (!block_shutdown_signals(shutdown_signals))
  {
    return 1;
  }

  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor, "controller_manager");

  const bool use_sim_time = cm->get_parameter_or("use_sim_time", false);
  const bool has_realtime = realtime_tools::has_realtime_kernel();
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", has_realtime);
  if (lock_memory)
  {
    const auto lock_result = realtime_tools::lock_memory();
    if (!lock_result.first)
    {
      RCLCPP_WARN(cm->get_logger(), "Unable to lock the memory: '%s'", lock_result.second.c_str());
    }
  }

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());
  const int thread_priority =
    cm->get_parameter_or<int>("thread_priority", kDefaultSchedPriority);
  RCLCPP_INFO(
    cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(),
    thread_priority);

  std::atomic_bool keep_running{true};
  std::thread cm_thread(
    [cm, thread_priority, use_sim_time, &keep_running]()
    {
      rclcpp::Parameter cpu_affinity_param;
      if (cm->get_parameter("cpu_affinity", cpu_affinity_param))
      {
        std::vector<int> cpus;
        if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          cpus = {static_cast<int>(cpu_affinity_param.as_int())};
        }
        else if (
          cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
          for (const auto cpu : cpu_affinity_param.as_integer_array())
          {
            cpus.push_back(static_cast<int>(cpu));
          }
        }
        const auto affinity_result = realtime_tools::set_current_thread_affinity(cpus);
        if (!affinity_result.first)
        {
          RCLCPP_WARN(
            cm->get_logger(), "Unable to set the CPU affinity : '%s'",
            affinity_result.second.c_str());
        }
      }

      if (!realtime_tools::configure_sched_fifo(thread_priority))
      {
        RCLCPP_WARN(
          cm->get_logger(),
          "Could not enable FIFO RT scheduling policy: with error number <%i>(%s).", errno,
          std::strerror(errno));
      }
      else
      {
        RCLCPP_INFO(
          cm->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.",
          thread_priority);
      }

      const auto period =
        std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());
      const auto cm_now = std::chrono::nanoseconds(cm->now().nanoseconds());
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
        next_iteration_time{cm_now};
      rclcpp::Time previous_time = cm->now();

      while (keep_running.load(std::memory_order_acquire))
      {
        const auto current_time = cm->now();
        const auto measured_period = current_time - previous_time;
        previous_time = current_time;

        cm->read(cm->now(), measured_period);
        cm->update(cm->now(), measured_period);
        cm->write(cm->now(), measured_period);

        next_iteration_time += period;
        if (use_sim_time)
        {
          cm->get_clock()->sleep_until(current_time + period);
        }
        else
        {
          std::this_thread::sleep_until(next_iteration_time);
        }
      }
    });

  std::thread signal_thread(
    [&shutdown_signals, &keep_running, &executor]()
    {
      const timespec timeout{0, kSignalPollNanoseconds};
      while (keep_running.load(std::memory_order_acquire))
      {
        const int received = sigtimedwait(&shutdown_signals, nullptr, &timeout);
        if (received == SIGINT || received == SIGTERM)
        {
          keep_running.store(false, std::memory_order_release);
          executor->cancel();
          return;
        }
        if (received < 0 && errno != EAGAIN && errno != EINTR)
        {
          keep_running.store(false, std::memory_order_release);
          executor->cancel();
          return;
        }
      }
    });

  executor->add_node(cm);
  executor->spin();
  keep_running.store(false, std::memory_order_release);
  executor->cancel();
  cm_thread.join();
  signal_thread.join();

  // ControllerManager's pre-shutdown callback now runs without a concurrent
  // hardware read/update/write cycle.
  rclcpp::shutdown();
  return 0;
}
