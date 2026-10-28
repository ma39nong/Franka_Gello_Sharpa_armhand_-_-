from __future__ import annotations

import socket
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from teleop_core.contract import (
    WUJI_COMMAND_TOPIC,
    WUJI_LEFT_JOINT_NAMES,
    WUJI_RIGHT_JOINT_NAMES,
    WUJI_STATE_TOPIC,
    WUJI_TELEMETRY_STATUS_TOPIC,
)
from teleop_interfaces.msg import HandTelemetryStatus

# The Operator and ROS receiver intentionally share one SDK-free codec.  A
# symlink build resolves this source path directly; the fallback also supports
# invoking the node from a non-symlink install inside this repository.
try:
    from teleop_runtime.hand_telemetry import (
        PacketTracker,
        VelocityEstimator,
        decode_packet,
    )
except ModuleNotFoundError:
    repository = Path(__file__).resolve().parents[4]
    sys.path.insert(0, str(repository))
    from teleop_runtime.hand_telemetry import (  # type: ignore[no-redef]
        PacketTracker,
        VelocityEstimator,
        decode_packet,
    )


EXPECTED_JOINT_NAMES = {
    "left": WUJI_LEFT_JOINT_NAMES,
    "right": WUJI_RIGHT_JOINT_NAMES,
}


def _set_time(message, timestamp_ns: int) -> None:
    message.sec = int(timestamp_ns) // 1_000_000_000
    message.nanosec = int(timestamp_ns) % 1_000_000_000


class HandTelemetryReceiver(Node):
    """UDP receiver that has publishers only and cannot command hardware."""

    def __init__(self) -> None:
        super().__init__("teleop_hand_telemetry_receiver")
        self.declare_parameter("bind_host", "127.0.0.1")
        self.declare_parameter("bind_port", 5602)
        self.declare_parameter("max_staleness_ms", 150.0)
        host = str(self.get_parameter("bind_host").value)
        port = int(self.get_parameter("bind_port").value)
        staleness_ms = float(self.get_parameter("max_staleness_ms").value)
        if not host or not 0 < port < 65536 or staleness_ms <= 0.0:
            raise ValueError("invalid hand telemetry receiver parameters")

        self.command_publishers = {
            side: self.create_publisher(
                JointState, WUJI_COMMAND_TOPIC.format(side=side), 10
            )
            for side in ("left", "right")
        }
        self.state_publishers = {
            side: self.create_publisher(
                JointState, WUJI_STATE_TOPIC.format(side=side), 10
            )
            for side in ("left", "right")
        }
        self.status_publisher = self.create_publisher(
            HandTelemetryStatus, WUJI_TELEMETRY_STATUS_TOPIC, 10
        )
        self.tracker = PacketTracker(
            max_staleness_ns=int(staleness_ms * 1_000_000)
        )
        self.velocity = VelocityEstimator()
        self.invalid_packets = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind((host, port))
        self.timer = self.create_timer(0.002, self._drain)
        self.get_logger().info(
            f"read-only Wuji telemetry receiver on udp://{host}:{port}"
        )

    def _drain(self) -> None:
        for _ in range(64):
            try:
                data, _address = self.socket.recvfrom(65_535)
            except BlockingIOError:
                return
            except OSError as error:
                self.get_logger().error(f"telemetry socket failed: {error}")
                return
            received_ns = time.monotonic_ns()
            try:
                packet = decode_packet(data)
                expected = EXPECTED_JOINT_NAMES[packet.side]
                if packet.joint_names != expected:
                    raise ValueError(
                        f"{packet.side} Wuji joint-name/order mismatch"
                    )
                if not self.tracker.accept(
                    packet, received_monotonic_ns=received_ns
                ):
                    continue
                self._publish(packet, received_ns)
            except Exception as error:  # corrupt telemetry remains isolated
                self.invalid_packets += 1
                if self.invalid_packets <= 3 or self.invalid_packets % 100 == 0:
                    self.get_logger().warn(
                        f"discarding invalid hand telemetry: {error}"
                    )

    def _publish(self, packet, received_ns: int) -> None:
        command_freshness = _freshness(
            received_ns, packet.command_monotonic_ns
        )
        state_freshness = _freshness(received_ns, packet.state_monotonic_ns)

        if packet.command_valid and packet.command is not None:
            command = JointState()
            _set_time(command.header.stamp, packet.source_wall_time_ns)
            command.header.frame_id = (
                f"wuji:{packet.session_id}:{packet.sequence}:source_wall"
            )
            command.name = list(packet.joint_names)
            command.position = list(packet.command)
            self.command_publishers[packet.side].publish(command)

        velocity_source = "unavailable_no_state"
        published_state_valid = False
        if packet.state_valid and packet.state is not None:
            velocity = self.velocity.update(
                packet.side,
                packet.joint_names,
                packet.state,
                packet.state_monotonic_ns,
            )
            velocity_source = velocity.source
            if velocity.values is not None:
                state = JointState()
                _set_time(state.header.stamp, packet.source_wall_time_ns)
                state.header.frame_id = (
                    f"wuji:{packet.session_id}:{packet.sequence}:source_wall:"
                    "velocity=finite_difference"
                )
                state.name = list(packet.joint_names)
                state.position = list(packet.state)
                state.velocity = list(velocity.values)
                self.state_publishers[packet.side].publish(state)
                published_state_valid = True

        status = HandTelemetryStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "receiver_wall"
        status.session_id = packet.session_id
        status.sequence = packet.sequence
        status.side = packet.side
        _set_time(status.source_wall_stamp, packet.source_wall_time_ns)
        status.source_monotonic_ns = packet.source_monotonic_ns
        status.command_monotonic_ns = packet.command_monotonic_ns or 0
        status.state_monotonic_ns = packet.state_monotonic_ns or 0
        status.engaged = packet.engaged
        status.command_valid = packet.command_valid
        status.state_valid = published_state_valid
        status.command_freshness_sec = command_freshness
        status.state_freshness_sec = state_freshness
        status.state_velocity_source = velocity_source
        status.sender_dropped_packets = packet.sender_dropped_packets
        status.receiver_lost_packets = self.tracker.lost
        status.receiver_duplicate_packets = self.tracker.duplicates
        status.receiver_out_of_order_packets = self.tracker.out_of_order
        status.receiver_stale_packets = self.tracker.stale
        status.receiver_invalid_packets = self.invalid_packets
        self.status_publisher.publish(status)

    def destroy_node(self):
        try:
            self.socket.close()
        finally:
            return super().destroy_node()


def _freshness(received_ns: int, source_ns: int | None) -> float:
    if source_ns is None:
        return -1.0
    return max(0.0, (int(received_ns) - int(source_ns)) / 1_000_000_000.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandTelemetryReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

