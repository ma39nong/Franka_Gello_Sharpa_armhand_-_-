cmake_minimum_required(VERSION 3.16)
project(lychee_fr3_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(moveit_launch tests/test_moveit_launch.py)
endif()

ament_package()
