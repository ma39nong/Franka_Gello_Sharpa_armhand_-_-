cmake_minimum_required(VERSION 3.10)
project(lychee_teach_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED src/zero_effort_teach_controller.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(
  controller_interface lychee_teach_controllers.xml
)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES lychee_teach_controllers.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
