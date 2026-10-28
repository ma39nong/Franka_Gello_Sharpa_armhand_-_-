cmake_minimum_required(VERSION 3.10)
project(teleop_camera_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(camera_contract test/test_camera_contract.py)
endif()

ament_package()
