# O30i vendor source provenance

`linker_hand_ros2_sdk/LinkerHand/o30i_control.py` is copied without behavioral changes from the organization-internal Linker Hand O30i SDK staged in:

`/home/descfly/Downloads/litchi_hardware-main/src/litchi_hardware/hardware/linker/_vendor/o30i/linker_hand_o30i_control.py`

Source SHA-256: `f22fd051e8a5a47cc330d91e07971b30eff16d9034f320dd977543c52f292c73`.

The connected metal USB-CANFD adapter requires the corresponding x86_64 Ubuntu 22 vendor runtime, copied from the same source tree:

`src/litchi_hardware/hardware/linker/_vendor/o30i/lib/linux-x86_64-ubuntu22/libcanbus.so`

Runtime SHA-256: `6c7100a10415cbbde08d42d895ccdf4c9b680badf1f04e3322458037d320c543`.

The source implements the vendor HOP CAN-FD protocol. It is proprietary material supplied for internal hardware integration and must not be redistributed without authorization.

The project-owned ROS adapter and radians/tick calibration contract live outside the copied vendor file.
