#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace lychee_teach_controllers {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ZeroEffortTeachController : public controller_interface::ControllerInterface {
 public:
  CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string robot_type_;
  std::string arm_prefix_;
};

}  // namespace lychee_teach_controllers
