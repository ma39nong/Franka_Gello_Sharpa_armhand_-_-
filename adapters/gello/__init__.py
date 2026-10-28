"""GELLO adapter public API.

The implementation remains in the shared PICO-era Python distribution so
existing editable installs keep working; this package provides the stable
repository-level ownership boundary.
"""

from pico_bimanual_franka_teleop.gello_input import (
    DualGelloJointInput,
    load_gello_config,
)

__all__ = ["DualGelloJointInput", "load_gello_config"]
