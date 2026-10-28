#include "lychee_teach_controllers/zero_effort_teach_controller.hpp"

#include <cstdio>
#include <exception>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lychee_teach_controllers {

CallbackReturn ZeroEffortTeachController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3v2");
    auto_declare<std::string>("arm_prefix", "");
  } catch (const std::exception& exception) {
    std::fprintf(
        stderr, "ZeroEffortTeachController init failed: %s\n", exception.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZeroEffortTeachController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  if (!arm_prefix_.empty()) {
    arm_prefix_ += "_";
  }
  RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured zero-effort teach controller for joint prefix '%s%s'",
      arm_prefix_.c_str(), robot_type_.c_str());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ZeroEffortTeachController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int joint_index = 1; joint_index <= 7; ++joint_index) {
    config.names.push_back(
        arm_prefix_ + robot_type_ + "_joint" + std::to_string(joint_index) +
        "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
ZeroEffortTeachController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type ZeroEffortTeachController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return controller_interface::return_type::OK;
}

CallbackReturn ZeroEffortTeachController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace lychee_teach_controllers

PLUGINLIB_EXPORT_CLASS(
    lychee_teach_controllers::ZeroEffortTeachController,
    controller_interface::ControllerInterface)
