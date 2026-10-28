# teleop_data_collector

This package is a read-only ROS graph consumer. It shells out to
`ros2 bag record`, validates the stopped bag, and never publishes a robot or
hand command. Conversion is exposed as a separate manual executable and is
never started by the recorder.

It does not replace `teleop_data` (the O30i/G20 recorder). The two packages
stay independent and must not subscribe to each other's camera/hand topics.

The implementation was selectively adapted from the Pico repository's already
gateway-adapted collector, then renamed to this repository's GELLO recording
config (`record_gello.yaml`, `bags/gello`, `teleoperator=gello`).

