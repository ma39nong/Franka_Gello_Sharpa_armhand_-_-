# Orbbec retained-source provenance

The retained `orbbec_camera` and `orbbec_camera_msgs` packages were copied from
`/home/user/llx/harvest_ws/teleop/ros2/src/vendor` at Harvest commit
`962137571e5c91e7c4f9cf34952233e3365943eb`. Their notices identify upstream
OrbbecSDK ROS 2 revision `d60ab1812eb080fd8bb75699bff40d1d7e5f269f` and bundled
SDK 2.8.6.

Retained license hashes:

- `orbbec_camera/LICENSE`: `1a325fd390b54e7b142d31dfcbfa4ed4621b83d2ec8e3b9fad054d491b3a4c49`
- `orbbec_camera/NOTICE`: originally `b8d271920bf65d6e87190d7f3fe5bd809df806a4edc5ceb19005ee1994475a7f`; only its broken local provenance path was corrected after import.
- Orbbec SDK EULA: `343a1a440717719742c9ee30b3f9afa9f23c16289873ed6122411395b8e89500`
- `orbbec_camera_msgs/LICENSE`: `1a325fd390b54e7b142d31dfcbfa4ed4621b83d2ec8e3b9fad054d491b3a4c49`
- `orbbec_camera_msgs/NOTICE`: originally `54988e3a0fe6306caa48a8a5466d36b4248a571477ab55a350a129abccc7564d`; only its broken local provenance path was corrected after import.

Local changes are limited to making a normal workspace build skip the bundled
SDK runtime, adding an explicit CMake acceptance option, and requiring a
separate runtime environment gate. Neither gate represents acceptance on the
user's behalf.

This repository already vendors a single-camera Orbbec driver in the Docker
image for compose service `orbbec` (`/camera/...` topics). The Harvest packages
under `ros_ws/src/orbbec_camera*` therefore stay `COLCON_IGNORE` unless
`./ops/setup/build.sh --accept-orbbec-eula` temporarily un-ignores them. That
keeps default GELLO+MANUS+Wuji teleop from overlaying the existing camera
service.


