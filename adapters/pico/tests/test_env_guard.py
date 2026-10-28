import sys
from types import SimpleNamespace

from pico_bimanual_franka_teleop.env_guard import _reexec_argv


def test_reexec_keeps_module_invocation(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["/tmp/teleop_runtime/cli.py", "--hand-source", "wuji"])
    main = SimpleNamespace(__spec__=SimpleNamespace(name="teleop_runtime.cli"))

    assert _reexec_argv(main) == [
        sys.executable,
        "-m",
        "teleop_runtime.cli",
        "--hand-source",
        "wuji",
    ]


def test_reexec_keeps_script_invocation(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["/tmp/analyze_follow_log.py", "log.jsonl"])
    main = SimpleNamespace(__spec__=None)

    assert _reexec_argv(main) == [
        sys.executable,
        "/tmp/analyze_follow_log.py",
        "log.jsonl",
    ]
