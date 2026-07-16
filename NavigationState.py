"""
NavigationState.py

Simulation-side navigation state for Vanguard X.

This module represents the platform attitude independently of the PTZ.
Initially it provides heading only, but the interface already includes
pitch and roll for future moving-platform support.
"""

from dataclasses import dataclass, field
import time


def wrap360(angle_deg: float) -> float:
    return float(angle_deg) % 360.0


@dataclass
class PlatformAttitude:
    TimestampSec: float = field(default_factory=time.time)
    HeadingTrueDeg: float = 0.0
    PitchDeg: float = 0.0
    RollDeg: float = 0.0
    Valid: bool = True


class SimulatedNavigationSource:
    """
    Simple navigation simulator.

    Modes:
      - Fixed heading
      - Constant turn rate
    """

    def __init__(self,
                 initial_heading_deg: float = 0.0,
                 turn_rate_deg_per_sec: float = 0.0):
        self._heading = wrap360(initial_heading_deg)
        self._turn_rate = float(turn_rate_deg_per_sec)
        self._last_time = time.time()

    def set_heading(self, heading_deg: float):
        self._heading = wrap360(heading_deg)
        self._last_time = time.time()

    def set_turn_rate(self, turn_rate_deg_per_sec: float):
        self._update()
        self._turn_rate = float(turn_rate_deg_per_sec)

    def _update(self):
        now = time.time()
        dt = now - self._last_time
        self._heading = wrap360(self._heading + self._turn_rate * dt)
        self._last_time = now

    def get_attitude(self) -> PlatformAttitude:
        self._update()
        return PlatformAttitude(
            TimestampSec=self._last_time,
            HeadingTrueDeg=self._heading,
            PitchDeg=0.0,
            RollDeg=0.0,
            Valid=True,
        )
