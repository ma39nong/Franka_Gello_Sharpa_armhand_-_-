"""Hardware backends copied and adapted from wuji-retargeting."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import numpy as np


@dataclass(frozen=True)
class WujiHand2JointDiagnostic:
    """One Hand 2 diagnostic entry normalized to device joint index."""

    joint_index: int
    node_id: int
    current_a: float
    bus_voltage_v: float
    temperature_c: float
    error_code: int
    ext_state: int
    ext_state_name: str
    position_limit_active: bool
    velocity_limit_active: bool
    current_limit_active: bool
    comm_response_rate_pct: int
    comm_timeout_total: int


def _hand2_joint_index(node_id: int) -> int:
    finger, joint = divmod(int(node_id) - 1, 5)
    if node_id <= 0 or finger >= 5 or joint >= 4:
        raise ValueError(f"Wuji Hand 2 reported invalid joint id {node_id}")
    return finger * 4 + joint


def _hand2_diagnostics(frame: Any) -> dict[int, WujiHand2JointDiagnostic]:
    """Normalize a possibly unordered diagnostic frame by device joint index."""
    entries = tuple(frame.joints)
    if int(frame.num_joints) != len(entries):
        raise ValueError("Wuji Hand 2 diagnostic count does not match its entries")
    diagnostics: dict[int, WujiHand2JointDiagnostic] = {}
    for entry in entries:
        node_id = int(entry.nid)
        index = _hand2_joint_index(node_id)
        if index in diagnostics:
            raise ValueError(f"Wuji Hand 2 reported duplicate joint id {node_id}")
        status = entry.status_word
        diagnostics[index] = WujiHand2JointDiagnostic(
            joint_index=index,
            node_id=node_id,
            current_a=float(entry.current),
            bus_voltage_v=float(entry.vbus_v_fb),
            temperature_c=float(entry.mcu_temp_c_fb),
            error_code=int(entry.error_code_current),
            ext_state=int(status.ext_state),
            ext_state_name=str(status.ext_state_name),
            position_limit_active=bool(status.position_limit_active),
            velocity_limit_active=bool(status.velocity_limit_active),
            current_limit_active=bool(status.current_limit_active),
            comm_response_rate_pct=int(entry.comm_response_rate_pct),
            comm_timeout_total=int(entry.comm_timeout_total),
        )
    return diagnostics


class WujiHandBackend:
    """Original USB Wuji Hand backend."""

    def __init__(self, serial: str = "") -> None:
        try:
            import wujihandpy
        except ImportError as error:
            raise RuntimeError(
                "wujihandpy is required for model=wuji_hand"
            ) from error
        self._sdk = wujihandpy
        self._hand = (
            wujihandpy.Hand(serial_number=serial)
            if serial
            else wujihandpy.Hand()
        )
        self._hand.write_joint_enabled(True)
        self._controller = self._hand.realtime_controller(
            enable_upstream=False,
            filter=wujihandpy.filter.LowPass(cutoff_freq=5.0),
        )
        time.sleep(0.5)

    def send(self, qpos: np.ndarray) -> None:
        values = np.asarray(qpos, dtype=np.float64)
        if values.shape != (20,) or not np.isfinite(values).all():
            raise ValueError("Wuji Hand command must be 20 finite positions")
        self._controller.set_joint_target_position(values.reshape(5, 4))

    def close(self) -> None:
        if getattr(self, "_hand", None) is not None:
            self._hand.write_joint_enabled(False)


def _set_with_retry(description: str, operation, attempts: int = 3) -> None:
    for attempt in range(attempts):
        try:
            operation()
            return
        except (AttributeError, TypeError):
            raise
        except Exception:
            if attempt + 1 == attempts:
                raise
            print(
                f"Wuji Hand 2 {description} timed out; retrying "
                f"({attempt + 1}/{attempts})"
            )
            time.sleep(0.6)


def _hand2_feedback_positions(frame: Any) -> np.ndarray:
    """Return a complete Hand 2 feedback frame in firmware command order."""
    entries = tuple(frame.joints)
    if int(frame.num_joints) != len(entries):
        raise ValueError("Wuji Hand 2 feedback count does not match its entries")
    positions: dict[int, float] = {}
    for entry in entries:
        node_id = int(entry.nid)
        index = _hand2_joint_index(node_id)
        if index in positions:
            raise ValueError(f"Wuji Hand 2 reported duplicate joint id {node_id}")
        positions[index] = float(entry.position)
    if set(positions) != set(range(20)):
        missing = sorted(set(range(20)).difference(positions))
        raise RuntimeError(f"Wuji Hand 2 feedback is missing joints {missing}")
    values = np.asarray([positions[index] for index in range(20)], dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError("Wuji Hand 2 feedback contains non-finite positions")
    return values


def _connect_hand2(manager: Any, sdk: Any, *, address: str, side: str) -> Any:
    """Connect one Hand 2 without exposing its SDK bridge on FCI networks.

    The vendor SDK defaults ``enable_bridge`` to true.  That mode starts a
    local Zenoh router for other *processes* to attach to the same hand, and
    advertises a listener on every host address.  The arm teleop owns both
    hands from this single process, so that extra bridge is neither needed nor
    safe on a host whose Franka FCI NICs must stay dedicated to 1 kHz control.
    Direct UDP traffic to the explicitly supplied Hand 2 address is unchanged.
    """
    options = sdk.ConnectOptions(enable_bridge=False)
    return manager.connect(
        address=address,
        device_name=f"wuji_hand_2_{side}",
        options=options,
    )


class WujiHand2Backend:
    """Network Wuji Hand 2 backend using the vendor SDK."""

    def __init__(
        self,
        *,
        side: str,
        address: str,
        kp: float,
        kd: float,
        current_limit: float,
        auto_enable: bool = True,
    ) -> None:
        try:
            import wuji_sdk
            from wuji_sdk import SdkManager
        except ImportError as error:
            raise RuntimeError(
                "wuji-sdk>=0.10 is required for model=wuji_hand_2"
            ) from error
        if not address:
            raise ValueError(
                f"{side} Wuji Hand 2 requires an explicit address; "
                f"pass --wuji-{side}-address IP:PORT"
            )
        self._sdk = wuji_sdk
        self._manager = SdkManager.instance()
        # device_name is a local SdkManager alias, not the product type; it must
        # be unique when both hands are connected in one process.
        self._hand: Any = _connect_hand2(
            self._manager,
            wuji_sdk,
            address=address,
            side=side,
        )
        self._publisher: Any = None
        self._state_subscription: Any = None
        self._diagnostic_subscription: Any = None
        self._enabled = False
        self._last_command_position: np.ndarray | None = None
        self._kp_values = np.full(20, float(kp), dtype=np.float64)
        self._kd_values = np.full(20, float(kd), dtype=np.float64)
        self._current_limit = float(current_limit)
        try:
            reported_side = str(self._hand.handedness().get()).lower()
            if reported_side != side:
                raise RuntimeError(
                    f"Wuji hand at {address} reports {reported_side}, expected {side}"
                )
            online = int(self._hand.online_joints_count().get())
            if online != 20:
                raise RuntimeError(
                    f"{side} Wuji Hand 2 has {online}/20 online joints"
                )
            time.sleep(0.5)
            self._JointCommand = wuji_sdk.JointCommand
            self._state_subscription = self._hand.joint_states().subscribe()
            self._diagnostic_subscription = (
                self._hand.joint_diagnostics().subscribe()
            )
            if auto_enable:
                self.enable()
            else:
                self._hand.disable()
                print(f"{side} Wuji Hand 2 connected at {address} (motors disabled)")
        except BaseException:
            self.close()
            raise

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        if self._enabled:
            return
        _set_with_retry(
            "effort_limit",
            lambda: self._hand.effort_limit().set(self._current_limit),
        )
        _set_with_retry(
            "mit_params",
            lambda: self._hand.mit_params().set(list(self.mit_gains())),
        )
        self._hand.enable()
        self._wait_until_enabled()
        if self._publisher is None:
            self._publisher = self._hand.joint_command().publish()
        self._enabled = True
        print(
            "Wuji Hand 2 enabled: "
            f"kp={self._format_parameter(self._kp_values)}, "
            f"kd={self._format_parameter(self._kd_values)}, "
            f"current_limit={self._current_limit:g}A"
        )

    @staticmethod
    def _format_parameter(values: np.ndarray) -> str:
        if np.allclose(values, values[0]):
            return f"{values[0]:g}"
        return "per-joint"

    def mit_gains(self) -> tuple[tuple[float, float], ...]:
        return tuple(
            (float(kp), float(kd))
            for kp, kd in zip(self._kp_values, self._kd_values, strict=True)
        )

    def set_mit_gains(
        self, *, kp: float, kd: float, joint_index: int | None = None
    ) -> tuple[tuple[float, float], ...]:
        kp_value = float(kp)
        kd_value = float(kd)
        if (
            not np.isfinite((kp_value, kd_value)).all()
            or kp_value < 0.0
            or kd_value < 0.0
        ):
            raise ValueError("Wuji Hand 2 MIT gains must be finite and non-negative")
        kp_values = self._kp_values.copy()
        kd_values = self._kd_values.copy()
        if joint_index is None:
            kp_values.fill(kp_value)
            kd_values.fill(kd_value)
        else:
            index = int(joint_index)
            if not 0 <= index < 20:
                raise ValueError("Wuji Hand 2 joint index must be in 0..19")
            kp_values[index] = kp_value
            kd_values[index] = kd_value
        params = [
            (float(kp_item), float(kd_item))
            for kp_item, kd_item in zip(kp_values, kd_values, strict=True)
        ]
        _set_with_retry(
            "mit_params", lambda: self._hand.mit_params().set(params)
        )
        self._kp_values = kp_values
        self._kd_values = kd_values
        return self.mit_gains()

    def disable(self) -> None:
        if getattr(self, "_hand", None) is not None:
            self._hand.disable()
        self._enabled = False
        self._last_command_position = None

    def _wait_until_enabled(self) -> None:
        deadline = time.monotonic() + 5.0
        subscription = self._diagnostic_subscription
        while time.monotonic() < deadline:
            time.sleep(0.2)
            frame = subscription.recv()
            if frame is None:
                continue
            live = [entry for entry in frame.joints if entry.vbus_v_fb > 0.5]
            if live and all(entry.status_word.ext_state == 2 for entry in live):
                return
        self._hand.disable()
        raise RuntimeError("Wuji Hand 2 did not reach Enabled state within 5 s")

    def send(self, qpos: np.ndarray) -> None:
        values = np.asarray(qpos, dtype=np.float64)
        if values.shape != (20,) or not np.isfinite(values).all():
            raise ValueError("Wuji Hand 2 command must be 20 finite positions")
        if not self._enabled or self._publisher is None:
            raise RuntimeError("Wuji Hand 2 motors are disabled")
        commands = [
            self._JointCommand(float(position), 0.0, 0.0)
            for position in values
        ]
        self._publisher.send(commands)
        self._last_command_position = values.copy()

    @property
    def last_command_position(self) -> np.ndarray | None:
        command = self._last_command_position
        return None if command is None else command.copy()

    def read_position(self) -> np.ndarray | None:
        """Drain feedback and return the newest complete actual-position frame."""
        subscription = self._state_subscription
        if subscription is None:
            return None
        latest = None
        while (frame := subscription.recv()) is not None:
            latest = _hand2_feedback_positions(frame)
        return latest

    def read_diagnostics(self) -> dict[int, WujiHand2JointDiagnostic] | None:
        """Drain diagnostics and return the newest frame indexed by joint."""
        subscription = self._diagnostic_subscription
        if subscription is None:
            return None
        latest = None
        while (frame := subscription.recv()) is not None:
            latest = _hand2_diagnostics(frame)
        return latest

    def close(self) -> None:
        diagnostic_subscription = getattr(self, "_diagnostic_subscription", None)
        if diagnostic_subscription is not None:
            try:
                diagnostic_subscription.close()
            except Exception:
                pass
            self._diagnostic_subscription = None
        subscription = getattr(self, "_state_subscription", None)
        if subscription is not None:
            try:
                subscription.close()
            except Exception:
                pass
            self._state_subscription = None
        publisher = getattr(self, "_publisher", None)
        if publisher is not None:
            try:
                publisher.close()
            except Exception:
                pass
            self._publisher = None
        hand = getattr(self, "_hand", None)
        if hand is not None:
            try:
                self.disable()
            except Exception:
                pass
            try:
                hand.disconnect()
            except Exception:
                pass
            self._hand = None
