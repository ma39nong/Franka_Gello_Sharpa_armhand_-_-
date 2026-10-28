"""Interactively identify what each G20 command slot physically does.

Commands the vendor slots directly, bypassing retargeting entirely, so the answers
describe the hand and the vendor convention rather than anything in this
repository. Reads `/cb_{side}_hand_state` back after every move, so a slot that
does not respond shows up in the numbers without the observer having to notice it.

Every question replays its own motion on request. A demonstration that the
observer missed is the normal case, not an exceptional one, so `r` repeats.

Two facts are already settled on the left hand and are no longer asked:

  * 0 is fully flexed and 255 is extended, confirmed by driving slot 1 to 0 and
    seeing the index base knuckle curl while the outer joints stayed straight,
    which also confirms base and tip are separate motors;
  * the vendor's documented slot order for the four fingers is correct.

Both were verified by observation and are recorded in the results for
completeness. What remains cannot be derived from the vendor's radian tables,
because those describe its internal joint convention and say nothing about its
relationship to the L20 URDF.

Run the vendor driver first, and do NOT run the bridge at the same time: both
publish to the same control topic and would fight.

    ros2 run linker_hand_ros2_sdk linker_hand_sdk --ros-args \\
      -p hand_type:=left -p hand_joint:=G20 -p can:=can0 -p is_touch:=false
    ros2 run linker_hand_bridge slot_probe --ros-args -p side:=left
"""

from __future__ import annotations

import json
import sys
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .core import (
    COMMAND_SLOTS,
    G20_JOINT_NAMES,
    validate_hand_state,
)

FLEXION_SLOTS = (0, 1, 2, 3, 4, 15, 16, 17, 18, 19)
ABDUCTION_SLOTS = (6, 7, 8, 9)
RESERVED = (11, 12, 13, 14)
SIDE_LABEL = {"left": "左手", "right": "右手"}
REPEAT_KEY = "r"


def neutral_pose() -> list[float]:
    """Fingers extended, abduction mid-range."""
    pose = [128.0] * COMMAND_SLOTS
    for slot in FLEXION_SLOTS:
        pose[slot] = 255.0
    for slot in RESERVED:
        pose[slot] = 0.0
    return pose


class SlotProbe(Node):
    def __init__(self) -> None:
        super().__init__("slot_probe")
        self.declare_parameter("side", "left")
        self.declare_parameter("speed", 120)
        self.declare_parameter("settle", 2.0)
        self.declare_parameter("cycles", 2)
        self.side = str(self.get_parameter("side").value)
        if self.side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        self.speed = int(self.get_parameter("speed").value)
        self.settle = float(self.get_parameter("settle").value)
        self.cycles = max(1, int(self.get_parameter("cycles").value))

        self.state: tuple[float, ...] | None = None
        self.create_subscription(
            JointState, f"/cb_{self.side}_hand_state", self._on_state, 10
        )
        self.command_publisher = self.create_publisher(
            JointState, f"/cb_{self.side}_hand_control_cmd", 10
        )
        self.setting_publisher = self.create_publisher(
            String, "/cb_hand_setting_cmd", 10
        )

    def _on_state(self, message: JointState) -> None:
        validated = validate_hand_state(message.position)
        if validated is not None:
            self.state = validated

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_state(self, timeout: float = 15.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state is not None:
                return True
        return False

    def set_speed(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "setting_cmd": "set_speed",
                "params": {"hand_type": self.side, "speed": [self.speed] * 5},
            }
        )
        self.setting_publisher.publish(message)
        self.spin_for(1.0)

    def move(self, pose: list[float]) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(G20_JOINT_NAMES)
        message.position = [float(value) for value in pose]
        # Republish a few times: the driver consumes one pending command per tick.
        for _ in range(3):
            self.command_publisher.publish(message)
            self.spin_for(0.1)
        self.spin_for(self.settle)

    def show(self, label: str, pose: list[float], slots) -> None:
        """Announce, move, then print commanded against measured."""
        print(f"      {label}")
        self.move(pose)
        measured = self.state
        if measured is None:
            print("        没有读到状态")
            return
        print(
            "        "
            + "  ".join(
                f"槽位{s}: 指令={pose[s]:.0f} 实测={measured[s]:.0f}" for s in slots
            )
        )

    def alternate(self, label_a: str, pose_a, label_b: str, pose_b, slots) -> None:
        """Alternate between two poses so the difference is easy to catch."""
        for cycle in range(self.cycles):
            print(f"    第 {cycle + 1}/{self.cycles} 轮")
            self.show(label_a, pose_a, slots)
            self.show(label_b, pose_b, slots)


def ask(question: str, options: dict[str, str]) -> str:
    print()
    print(f"  >>> {question}")
    for key, text in options.items():
        print(f"        [{key}] {text}")
    while True:
        try:
            answer = input("      请输入字母: ").strip().lower()
        except EOFError:
            print("\n  读不到输入，请在交互式终端里运行")
            raise SystemExit(2)
        if answer in options:
            return answer
        print(f"      请输入这几个字母之一: {sorted(options)}")


def demo_and_ask(
    demo: Callable[[], None], question: str, options: dict[str, str]
) -> str:
    """Show the motion, ask, and replay as many times as the observer wants."""
    choices = {**options, REPEAT_KEY: "没看清，再做一遍"}
    while True:
        demo()
        answer = ask(question, choices)
        if answer != REPEAT_KEY:
            return answer
        print()
        print("    好，再做一遍。")


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = SlotProbe()
    label = SIDE_LABEL[node.side]
    findings: dict[str, str] = {
        "0是弯曲255是伸直": "已确认",
        "厂商槽位顺序": "已确认正确",
        "各关节机械上独立": "已确认",
    }
    try:
        print(f"正在测试{label}。每一题看不清都可以按 r 让它重做。")
        print("注意：bridge 不能同时运行，它会和本程序抢同一个话题。")
        if not node.wait_for_state():
            print("没有收到手的状态。厂商驱动启动了吗？", file=sys.stderr)
            return 1
        node.set_speed()

        base = neutral_pose()
        print()
        print("=== 先回到中位姿态：手指伸直，外展居中 ===")
        node.move(base)

        # ------------------------------------------------------ abduction direction
        print()
        print("=" * 70)
        print("外展极性标定")
        print("=" * 70)
        print("  只有【食指】的外展槽位会动，其他三根手指的指令保持不变。")
        print("  请只看食指往哪一侧偏：靠近拇指，还是靠近小指。")
        print("  上一轮把四个槽位设成同一个值是我出的题有问题：每个槽位控制的是")
        print("  那根手指自己的侧向角度，四根方向相同，所以同时给同一个值必然是")
        print("  整只手同向摆动、指缝不变。要看方向，只能单独动一根。")

        index_low = list(base)
        index_low[6] = 0.0
        index_high = list(base)
        index_high[6] = 255.0

        def index_demo() -> None:
            node.alternate(
                "食指外展槽位 = 0 ......", index_low,
                "食指外展槽位 = 255 ....", index_high,
                (6,),
            )

        answer = demo_and_ask(
            index_demo,
            "槽位设为 255 时，食指偏向【拇指那一侧】还是【小指那一侧】？",
            {
                "p": "偏向小指那一侧",
                "t": "偏向拇指那一侧",
                "n": "食指没有侧向移动",
            },
        )
        findings["食指255偏向"] = answer
        node.move(base)

        print()
        print("=" * 70)
        print(f"测试结果（{label}）")
        print("=" * 70)
        for key, value in findings.items():
            print(f"  {key:20s} {value}")
        print()
        # The shipped polarity was derived from this very observation on the left
        # hand, so a correctly configured hand now leans toward the thumb.
        expected = "t"
        if findings.get("食指255偏向") == expected:
            print("  外展极性与默认一致，与已标定的默认一致，无需改动")
        elif findings.get("食指255偏向") == "p":
            print("  外展极性与默认相反，请使用 abduction_invert:=true 翻回来")
        else:
            print("  食指没有侧向移动。可能是 ±0.17 rad 行程太窄看不出来，")
            print("  也可能这个槽位在本型号上没有接电机。可加 -p settle:=4.0 重测。")
        print()
        print("  若结论与当前配置不符，把这段贴回来即可。")
        return 0
    except KeyboardInterrupt:
        print("\n已被操作者中断")
        return 0
    finally:
        try:
            node.move(neutral_pose())
        except Exception:  # noqa: BLE001 - best effort on the way out
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
