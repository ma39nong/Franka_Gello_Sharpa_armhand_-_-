cmake_minimum_required(VERSION 3.16)
project(lychee_fr3_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes robots rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
