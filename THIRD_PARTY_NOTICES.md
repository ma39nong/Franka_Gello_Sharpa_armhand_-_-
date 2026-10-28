# Third-party notices

The realtime FR3 controller is adapted from Franka Robotics ROS 2 controller code and retains its Apache-2.0 license and NOTICE files in `ros_ws/src/franka_fr3_arm_controllers`.

The MuJoCo FR3 model includes Franka Robotics description assets. Their license and notice are retained in `adapters/pico/assets/dual_fr3/FRANKA_DESCRIPTION_LICENSE` and `FRANKA_DESCRIPTION_NOTICE`.

The PICO adapter includes the XRoboToolkit PC Service Python binding and its prebuilt Linux SDK library under `vendor/xrobotoolkit_sdk`. Its upstream license is retained alongside the source. The included native library targets x86-64 Linux; rebuild it from upstream for another architecture.

The VIVE Tracker adapter depends on the upstream `openvr` Python package and the locally installed SteamVR/OpenVR runtime. Those components remain governed by their respective upstream licenses and are not vendored in this repository.

The MANUS hand adapter includes MANUS CoreSDK 3.1.1 headers and its integrated x86-64 Linux library under `vendor/manus_sdk`. These proprietary files remain governed by the vendor agreement retained as `LICENSE.vendor`. The adapter source adds the attribution required for derivatives of vendor sample code.

The hand retargeting assets under `assets/linkerhand_l20` are LinkerHand L20 URDFs and meshes distributed by the vendor under Apache-2.0. Their upstream license is retained alongside the models.

The hand retargeting optimizer in `adapters/pico/src/pico_bimanual_franka_teleop/hand_retarget.py` and the G20 projection and slew limiter in `ros_ws/src/linker_hand_bridge/linker_hand_bridge/core.py` are adapted from the sibling WiLoR repository, which drove the same hand from monocular MANO reconstructions. They were copied rather than imported because sibling repositories are read-only references.

The container downloads pinned upstream releases of libfranka, franka_ros2, and franka_description during the build. Those components remain under their respective upstream licenses.

Harvest three-camera collection retains Orbbec SDK ROS 2 wrapper sources under
`ros_ws/src/orbbec_camera` and `ros_ws/src/orbbec_camera_msgs`, including their
Apache-2.0 LICENSE/NOTICE files and the bundled SDK 2.8.6 EULA in
`ros_ws/src/orbbec_camera/SDK`. Ordinary workspace builds skip that runtime.
Building or launching it requires an explicit operator-accepted gate; see
`data_collection/docs/ORBBEC_PROVENANCE.md`.

The optional dual-FR3 MoveIt packages, zero-effort teach controller, and
NiceGUI hand-control source under `ros_ws/src/lychee_fr3_*`,
`ros_ws/src/lychee_teach_controllers`, and `apps/hand_ui/barmate` were imported from
the local `lychee_barmate` repository at revision
`0b8722134d07b872130b40e8642789a35c515424`. The ROS package manifests declare
Apache-2.0 for the MoveIt packages. The source repository did not contain a
top-level license covering the Barmate application, so that code should be
treated as internal-source code unless its owner supplies different terms.

The optional Wuji retargeting implementation and robot assets under
`adapters/wuji` were imported from the local `wuji-retargeting` repository
at revision `66693c2f82f4b8ad40b6802620bab6488b41c18d`. Its upstream license and
the separate Wuji description license are retained as
`adapters/wuji/LICENSE` and `adapters/wuji/WUJI_DESCRIPTION_LICENSE`.
