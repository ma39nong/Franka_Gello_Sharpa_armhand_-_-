"""Camera backend selection for camera tools."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal, TypeAlias, cast

from barmate.hardware.cameras.orbbec import OrbbecCamera, list_orbbec_serials
from barmate.hardware.cameras.realsense import RealSenseCamera, list_realsense_serials

CameraBackendName: TypeAlias = Literal["realsense", "orbbec"]


@dataclass(frozen=True, slots=True)
class CameraBackend:
    """Runtime camera backend used by the camera tools."""

    name: CameraBackendName
    display_name: str
    camera_class: type[Any]
    list_serials: Callable[[], list[str]]


CAMERA_BACKENDS: dict[CameraBackendName, CameraBackend] = {
    "realsense": CameraBackend(
        name="realsense",
        display_name="RealSense",
        camera_class=RealSenseCamera,
        list_serials=list_realsense_serials,
    ),
    "orbbec": CameraBackend(
        name="orbbec",
        display_name="Orbbec",
        camera_class=OrbbecCamera,
        list_serials=list_orbbec_serials,
    ),
}


def camera_backend(name: str | None) -> CameraBackend:
    """Return the configured backend; default to RealSense for old callers."""

    backend_name = "realsense" if name is None else name.lower()
    if backend_name in CAMERA_BACKENDS:
        return CAMERA_BACKENDS[cast(CameraBackendName, backend_name)]
    choices = ", ".join(sorted(CAMERA_BACKENDS))
    raise ValueError(f"Unknown camera backend '{name}'. Choose one of: {choices}")


def backend_choices() -> tuple[str, ...]:
    """Return backend names accepted by the command-line tools."""

    return tuple(CAMERA_BACKENDS)


__all__ = [
    "CAMERA_BACKENDS",
    "CameraBackend",
    "CameraBackendName",
    "backend_choices",
    "camera_backend",
]
