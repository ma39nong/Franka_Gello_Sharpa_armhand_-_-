import argparse
import threading
import time
from pathlib import Path

import rclpy
from std_srvs.srv import Trigger

from .command_shell import run_command_shell
from .config import load_config
from .recorder import EpisodeRecorder


COMMANDS = {
    "/record": "start a new episode",
    "/stop": "stop recording and keep it pending",
    "/save": "save the pending episode",
    "/discard": "discard the pending episode",
    "/capture": "save the current measured joints as the initial pose",
    "/reset": "move both arms to the saved initial pose",
    "/status": "show recording, robot-state, and reset-service status",
    "/help": "show commands",
    "/quit": "exit",
}


class Operator:
    def __init__(self, node):
        self.node = node
        self.reset_client = node.create_client(Trigger, "/reset_to_initial_pose")
        self.capture_client = node.create_client(Trigger, "/capture_initial_pose")

    def _call(self, client, name, timeout):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                f"{name} service is unavailable after 10 seconds; "
                "start or restart franka-control and wait for reset=ready"
            )
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise RuntimeError(f"{name} timed out")
        result = future.result()
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def capture(self):
        return self._call(self.capture_client, "Initial-pose capture", 10.0)

    def reset(self):
        return self._call(self.reset_client, "Initial-pose reset", 120.0)

    def status(self):
        if self.node.recorder:
            recording = "recording"
        elif self.node.pending:
            recording = "pending"
        else:
            recording = "idle"
        discovered = dict(self.node.get_topic_names_and_types())
        arms = sum(
            f"/{side}/franka/joint_states" in discovered for side in ("left", "right")
        )
        required = [spec for spec in self.node.config.topics if spec.required]
        ready = sum(
            spec.type_name in discovered.get(spec.topic, []) for spec in required
        )
        reset = "ready" if self.reset_client.service_is_ready() else "offline"
        return (
            f"{recording} | arms={arms}/2 | "
            f"record_topics={ready}/{len(required)} | reset={reset}"
        )


def print_help():
    width = max(len(command) for command in COMMANDS)
    for command, description in COMMANDS.items():
        print(f"  {command.ljust(width)}  {description}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--qos", type=Path, required=True)
    return parser.parse_args()


def main():
    options = parse_args()
    config = load_config(options.config)
    rclpy.init()
    node = EpisodeRecorder(
        options.config,
        config.data_root,
        options.qos,
    )
    operator = Operator(node)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    def dispatch(command):
        if command == "/record":
            node.start()
            print("Recording.", flush=True)
        elif command == "/stop":
            node.stop()
            print("Stopped.", flush=True)
        elif command == "/save":
            print(f"Saved {node.save()}.", flush=True)
        elif command == "/discard":
            node.discard()
            print("Discarded.", flush=True)
        elif command == "/capture":
            print(operator.capture(), flush=True)
        elif command == "/reset":
            print(operator.reset(), flush=True)
        elif command == "/status":
            print(operator.status(), flush=True)
        elif command == "/help":
            print_help()
        elif command == "/quit":
            return False
        else:
            print(f"Unknown command: {command} (try /help)", flush=True)
        return True

    try:
        print_help()
        run_command_shell(
            COMMANDS,
            dispatch,
            operator.status,
            title="Dual-FR3 teleoperation operator",
            emergency_command="/stop",
        )
    finally:
        node.stop()
        rclpy.shutdown()
        spin_thread.join(timeout=5.0)
        node.destroy_node()
