cmake_minimum_required(VERSION 3.8)
project(orbbec_camera)

find_package(ament_cmake REQUIRED)

option(
  LIA_ENABLE_LICENSE_GATED_ORBBEC_RUNTIME
  "Build the bundled Orbbec SDK runtime after explicit EULA acceptance"
  OFF
)
if(NOT LIA_ENABLE_LICENSE_GATED_ORBBEC_RUNTIME)
  message(STATUS
    "Skipping the bundled Orbbec runtime. Review "
    "SDK/End User License Agreement.txt, then rebuild explicitly with "
    "-DLIA_ENABLE_LICENSE_GATED_ORBBEC_RUNTIME=ON after acceptance."
  )
  install(FILES LICENSE NOTICE "SDK/End User License Agreement.txt"
    DESTINATION share/${PROJECT_NAME}/licenses)
  ament_package()
  return()
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Werror
    -Wno-pedantic
    -Wno-array-bounds
    -Wno-stringop-overflow
  )
endif()

set(ros_dependencies
  ament_index_cpp
  backward_ros
  camera_info_manager
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  image_publisher
  image_transport
  message_filters
  nlohmann_json
  orbbec_camera_msgs
  rclcpp
  rclcpp_components
  rcutils
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

foreach(dependency IN LISTS ros_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  message(FATAL_ERROR "The retained Orbbec closure supports Linux x86_64 only")
endif()

set(orbbec_sdk_include "${CMAKE_CURRENT_SOURCE_DIR}/SDK/include")
set(orbbec_sdk_lib "${CMAKE_CURRENT_SOURCE_DIR}/SDK/lib/x64")

add_library(${PROJECT_NAME} SHARED
  src/d2c_viewer.cpp
  src/dynamic_params.cpp
  src/frame_timestamp_csv_logger.cpp
  src/image_publisher.cpp
  src/jpeg_decoder.cpp
  src/ob_camera_node.cpp
  src/ob_camera_node_driver.cpp
  src/ros_param_backend.cpp
  src/ros_service.cpp
  src/synced_imu_publisher.cpp
  src/utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${orbbec_sdk_include}>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} ${ros_dependencies})
target_link_directories(${PROJECT_NAME} PRIVATE "${orbbec_sdk_lib}")
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  OpenSSL::Crypto
  Threads::Threads
  ${OpenCV_LIBS}
  OrbbecSDK
  dl
  rt
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "orbbec_camera::OBCameraNodeDriver"
  EXECUTABLE orbbec_camera_node
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${orbbec_sdk_lib}"
  INSTALL_RPATH "$ORIGIN"
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY SDK/include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(
  DIRECTORY SDK/lib/x64/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)
install(
  FILES scripts/99-obsensor-libusb.rules
  DESTINATION share/${PROJECT_NAME}/udev
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${ros_dependencies})
ament_package()
