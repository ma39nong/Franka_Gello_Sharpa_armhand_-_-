"""Hardware-centric BarMate refactor package.

The new root-level package hosts low-level hardware interfaces and drivers while
coexisting with the legacy implementation under ``src/barmate`` during the
migration period.
"""

from __future__ import annotations

from pathlib import Path

_legacy_package = Path(__file__).resolve().parents[1] / "src" / "barmate"
if _legacy_package.is_dir():
    __path__.append(str(_legacy_package))

__all__ = ["core", "hardware", "teleop", "apps"]
