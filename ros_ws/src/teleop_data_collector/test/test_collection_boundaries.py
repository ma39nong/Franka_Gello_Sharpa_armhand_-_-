from pathlib import Path
import re


REPO_ROOT = Path(__file__).resolve().parents[4]
COLLECTION_ROOTS = (
    REPO_ROOT / "ros_ws/src/teleop_data_collector",
    REPO_ROOT / "ros_ws/src/teleop_hand_telemetry",
    REPO_ROOT / "ros_ws/src/teleop_camera_bringup",
    REPO_ROOT / "ops/run",
    REPO_ROOT / "data_collection",
    REPO_ROOT / "teleop_runtime/hand_telemetry.py",
)

FORBIDDEN_IMPORTS = (
    "pico_teleop_bridge",
    "teleop_core.safety_gateway",
    "teleop_core.safety",
    "DualFr3HardwareTeleop",
    "OperatorControlServer",
    "WujiHand2Backend",
    "WujiHandBackend",
    "WujiHandPipeline",
    "ManusBridge",
    " DualGelloJointInput",
)
FORBIDDEN_PUBLISH_TOPICS = (
    "/teleop/arm_commands",
    "/target_robot/joint_commands",
)
ALLOWED_SCRIPT_NAMES = {
    "start_recording.sh",
    "convert_recording.sh",
    "run_data_collection_container.sh",
}


def _iter_collection_text():
    for root in COLLECTION_ROOTS:
        if root.is_file():
            yield root, root.read_text(encoding="utf-8")
            continue
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            if "test" in path.parts and path.name.startswith("test_"):
                continue
            if path.suffix not in {".py", ".sh", ".yaml", ".md", ".xml"}:
                continue
            if root.name == "ops" or "ops/run" in str(root):
                if path.suffix == ".sh" and path.name not in ALLOWED_SCRIPT_NAMES:
                    continue
            yield path, path.read_text(encoding="utf-8")


def test_collection_packages_do_not_import_control_or_hardware_clients():
    offenders = []
    for path, text in _iter_collection_text():
        if path.suffix not in {".py", ".sh"}:
            continue
        for token in FORBIDDEN_IMPORTS:
            if token in text:
                offenders.append(f"{path.relative_to(REPO_ROOT)}: {token.strip()}")
    assert offenders == []


def test_collection_packages_do_not_publish_fr3_command_bus():
    offenders = []
    publish_pattern = re.compile(r"create_publisher|publish\(")
    for path, text in _iter_collection_text():
        if path.suffix != ".py":
            continue
        if not publish_pattern.search(text):
            continue
        for topic in FORBIDDEN_PUBLISH_TOPICS:
            if topic in text:
                offenders.append(f"{path.relative_to(REPO_ROOT)}: {topic}")
    assert offenders == []


def test_start_recording_script_keeps_collection_outside_control_stack():
    script = (REPO_ROOT / "ops/run/start_recording.sh").read_text(encoding="utf-8")
    container = (
        REPO_ROOT / "ops/run/run_data_collection_container.sh"
    ).read_text(encoding="utf-8")
    convert = (REPO_ROOT / "ops/run/convert_recording.sh").read_text(
        encoding="utf-8"
    )
    assert "start_wuji_teleop.sh" not in script
    assert "hand-control" in script
    assert "orbbec" in script
    assert "ORBBEC_SDK_LICENSE_ACCEPTED" in script
    assert "Git repository" in script
    assert "bags/gello" in script
    assert "record_gello.yaml" in container
    assert "convert_gello_lerobot_v2.yaml" in convert


def test_sharpa_collection_reuses_persistent_episode_controller():
    script = (REPO_ROOT / "ops/run/start_sharpa_arm_recording.sh").read_text(
        encoding="utf-8"
    )
    container = (
        REPO_ROOT / "ops/run/run_sharpa_camera_recording_container.sh"
    ).read_text(encoding="utf-8")

    assert "record_gello_sharpa.yaml" in script
    assert "sharpa_recording_postflight" not in script
    assert "ros2 bag record" not in script
    assert "docker compose run -d" in script
    assert "docker kill --signal=TERM" in script
    assert "rosbag_data_collector" in container
    assert "setsid" in container
    assert "camera_stream_health wait" in container
    assert "COLLECTION_OUTPUT_BAG" not in container
