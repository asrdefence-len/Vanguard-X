"""Timestamped navigation state for the Vanguard X radar system.

The navigation source represents the motion of the radar platform, independently
of the X6-60 antenna positioning unit.  Operational GPS/WT901 input and the
digital twin will eventually publish the same ``NavigationPose`` contract.

Coordinate and timing conventions
---------------------------------
* Geographic position is WGS-84 latitude, longitude and ellipsoidal altitude.
* Mission-local position and velocity are East-North-Up (ENU).
* Heading is degrees clockwise from true north.
* ``TimestampSec`` is Unix time at the navigation measurement epoch.
* The mission ENU origin is fixed for the life of a mission.

``PlatformAttitude`` and the original simulator methods remain available as a
compatibility interface for the current PointingManager integration.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Callable, Optional

from CoordinateFrames import (
    EnuPosition,
    EnuVelocity,
    GeodeticPosition,
    LocalEnuFrame,
    normalise_bearing_degrees,
)


def wrap360(angle_deg: float) -> float:
    """Compatibility alias for Vanguard's standard bearing normalisation."""

    return normalise_bearing_degrees(angle_deg)


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class PlatformAttitude:
    """Compatibility view used by the current pointing-control interface."""

    TimestampSec: float = field(default_factory=time.time)
    HeadingTrueDeg: float = 0.0
    PitchDeg: float = 0.0
    RollDeg: float = 0.0
    Valid: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "HeadingTrueDeg",
            wrap360(_finite(self.HeadingTrueDeg, "HeadingTrueDeg")),
        )
        _finite(self.TimestampSec, "TimestampSec")
        _finite(self.PitchDeg, "PitchDeg")
        _finite(self.RollDeg, "RollDeg")


@dataclass(frozen=True)
class NavigationPose:
    """Earth-referenced radar-platform pose at one measurement epoch."""

    TimestampSec: float
    SequenceNumber: int

    Geodetic: GeodeticPosition
    PositionEnu: EnuPosition
    VelocityEnu: EnuVelocity

    HeadingTrueDeg: float
    PitchDeg: float = 0.0
    RollDeg: float = 0.0
    YawRateDegPerSec: float = 0.0

    PositionValid: bool = True
    HeadingValid: bool = True
    VelocityValid: bool = True
    Source: str = "UNKNOWN"

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "HeadingTrueDeg",
            wrap360(_finite(self.HeadingTrueDeg, "HeadingTrueDeg")),
        )
        _finite(self.TimestampSec, "TimestampSec")
        _finite(self.PitchDeg, "PitchDeg")
        _finite(self.RollDeg, "RollDeg")
        _finite(self.YawRateDegPerSec, "YawRateDegPerSec")

        if int(self.SequenceNumber) < 0:
            raise ValueError("SequenceNumber must not be negative")
        object.__setattr__(self, "SequenceNumber", int(self.SequenceNumber))

    @property
    def Valid(self) -> bool:
        """True when the pose can Earth-reference a radar measurement."""

        return bool(self.PositionValid and self.HeadingValid)

    def age_sec(self, current_time_sec: Optional[float] = None) -> float:
        """Return non-negative age at ``current_time_sec``."""

        now = time.time() if current_time_sec is None else float(current_time_sec)
        return max(0.0, now - float(self.TimestampSec))

    def is_fresh(
        self,
        maximum_age_sec: float,
        current_time_sec: Optional[float] = None,
    ) -> bool:
        """Return whether the Earth-reference pose is valid and recent enough."""

        maximum_age = _finite(maximum_age_sec, "maximum_age_sec")
        if maximum_age < 0.0:
            raise ValueError("maximum_age_sec must not be negative")
        return self.Valid and self.age_sec(current_time_sec) <= maximum_age

    def as_attitude(self) -> PlatformAttitude:
        """Return the backwards-compatible attitude-only representation."""

        return PlatformAttitude(
            TimestampSec=self.TimestampSec,
            HeadingTrueDeg=self.HeadingTrueDeg,
            PitchDeg=self.PitchDeg,
            RollDeg=self.RollDeg,
            Valid=self.HeadingValid,
        )


class SimulatedNavigationSource:
    """Deterministic-capable navigation source for the Vanguard digital twin.

    The first increment supports fixed or constant-turn heading together with
    constant ENU velocity.  Supplying a controllable ``time_source`` makes the
    simulation deterministic without sleeping in tests.
    """

    def __init__(
        self,
        initial_heading_deg: float = 0.0,
        turn_rate_deg_per_sec: float = 0.0,
        *,
        mission_origin: Optional[GeodeticPosition] = None,
        initial_position: Optional[GeodeticPosition] = None,
        velocity_east_mps: float = 0.0,
        velocity_north_mps: float = 0.0,
        velocity_up_mps: float = 0.0,
        pitch_deg: float = 0.0,
        roll_deg: float = 0.0,
        source_name: str = "SIMULATED_NAVIGATION",
        time_source: Callable[[], float] = time.time,
    ):
        if not callable(time_source):
            raise TypeError("time_source must be callable")

        if mission_origin is None:
            mission_origin = initial_position or GeodeticPosition(0.0, 0.0, 0.0)
        if initial_position is None:
            initial_position = mission_origin

        self.MissionOrigin = mission_origin
        self.Frame = LocalEnuFrame(mission_origin)

        self._position_enu = self.Frame.to_enu(initial_position)
        self._velocity_enu = EnuVelocity(
            east_mps=_finite(velocity_east_mps, "velocity_east_mps"),
            north_mps=_finite(velocity_north_mps, "velocity_north_mps"),
            up_mps=_finite(velocity_up_mps, "velocity_up_mps"),
        )
        self._heading = wrap360(
            _finite(initial_heading_deg, "initial_heading_deg")
        )
        self._turn_rate = _finite(
            turn_rate_deg_per_sec,
            "turn_rate_deg_per_sec",
        )
        self._pitch = _finite(pitch_deg, "pitch_deg")
        self._roll = _finite(roll_deg, "roll_deg")
        self._source_name = str(source_name)
        self._time_source = time_source
        self._last_time = _finite(self._time_source(), "time_source result")
        self._sequence_number = 0

    def _now(self, current_time_sec: Optional[float] = None) -> float:
        return _finite(
            self._time_source() if current_time_sec is None else current_time_sec,
            "current_time_sec",
        )

    def _update(self, current_time_sec: Optional[float] = None) -> float:
        now = self._now(current_time_sec)
        dt = now - self._last_time
        if dt < 0.0:
            raise ValueError("navigation time cannot move backwards")

        if dt > 0.0:
            self._position_enu = EnuPosition(
                east_m=(
                    self._position_enu.east_m
                    + self._velocity_enu.east_mps * dt
                ),
                north_m=(
                    self._position_enu.north_m
                    + self._velocity_enu.north_mps * dt
                ),
                up_m=(
                    self._position_enu.up_m
                    + self._velocity_enu.up_mps * dt
                ),
            )
            self._heading = wrap360(self._heading + self._turn_rate * dt)
            self._last_time = now

        return now

    def set_heading(
        self,
        heading_deg: float,
        current_time_sec: Optional[float] = None,
    ) -> None:
        self._update(current_time_sec)
        self._heading = wrap360(_finite(heading_deg, "heading_deg"))

    def set_turn_rate(
        self,
        turn_rate_deg_per_sec: float,
        current_time_sec: Optional[float] = None,
    ) -> None:
        self._update(current_time_sec)
        self._turn_rate = _finite(
            turn_rate_deg_per_sec,
            "turn_rate_deg_per_sec",
        )

    def set_velocity_enu(
        self,
        east_mps: float,
        north_mps: float,
        up_mps: float = 0.0,
        current_time_sec: Optional[float] = None,
    ) -> None:
        self._update(current_time_sec)
        self._velocity_enu = EnuVelocity(
            east_mps=_finite(east_mps, "east_mps"),
            north_mps=_finite(north_mps, "north_mps"),
            up_mps=_finite(up_mps, "up_mps"),
        )

    def set_position_geodetic(
        self,
        position: GeodeticPosition,
        current_time_sec: Optional[float] = None,
    ) -> None:
        self._update(current_time_sec)
        self._position_enu = self.Frame.to_enu(position)

    def get_pose(
        self,
        current_time_sec: Optional[float] = None,
    ) -> NavigationPose:
        timestamp = self._update(current_time_sec)
        self._sequence_number += 1
        return NavigationPose(
            TimestampSec=timestamp,
            SequenceNumber=self._sequence_number,
            Geodetic=self.Frame.to_geodetic(self._position_enu),
            PositionEnu=self._position_enu,
            VelocityEnu=self._velocity_enu,
            HeadingTrueDeg=self._heading,
            PitchDeg=self._pitch,
            RollDeg=self._roll,
            YawRateDegPerSec=self._turn_rate,
            PositionValid=True,
            HeadingValid=True,
            VelocityValid=True,
            Source=self._source_name,
        )

    def get_attitude(
        self,
        current_time_sec: Optional[float] = None,
    ) -> PlatformAttitude:
        """Return the legacy attitude view used by PointingManager."""

        return self.get_pose(current_time_sec).as_attitude()


class CircularRouteNavigationSource:
    """Deterministic offshore circular-route navigation digital twin.

    The platform follows a circle in the mission ENU plane.  Position,
    velocity, true heading and yaw rate are derived from one analytic motion
    model, so the simulated navigation fields cannot drift out of agreement.

    ``initial_radial_bearing_deg`` is the true bearing from the circle centre
    to the platform at the initial time.  For example, zero degrees starts the
    platform at the north point of the circle.  Clockwise motion from that
    point initially heads east.
    """

    def __init__(
        self,
        *,
        mission_origin: GeodeticPosition,
        centre_east_m: float = 0.0,
        centre_north_m: float = 0.0,
        radius_m: float = 1000.0,
        speed_mps: float = 5.0,
        clockwise: bool = True,
        initial_radial_bearing_deg: float = 0.0,
        altitude_m: float = 0.0,
        pitch_deg: float = 0.0,
        roll_deg: float = 0.0,
        source_name: str = "SIMULATED_CIRCULAR_ROUTE",
        time_source: Callable[[], float] = time.time,
    ):
        if not callable(time_source):
            raise TypeError("time_source must be callable")

        radius = _finite(radius_m, "radius_m")
        speed = _finite(speed_mps, "speed_mps")
        if radius <= 0.0:
            raise ValueError("radius_m must be greater than zero")
        if speed < 0.0:
            raise ValueError("speed_mps must not be negative")

        self.MissionOrigin = mission_origin
        self.Frame = LocalEnuFrame(mission_origin)
        self.CentreEastM = _finite(centre_east_m, "centre_east_m")
        self.CentreNorthM = _finite(centre_north_m, "centre_north_m")
        self.RadiusM = radius
        self.SpeedMps = speed
        self.Clockwise = bool(clockwise)
        self.InitialRadialBearingDeg = wrap360(
            _finite(
                initial_radial_bearing_deg,
                "initial_radial_bearing_deg",
            )
        )
        self.AltitudeM = _finite(altitude_m, "altitude_m")
        self.PitchDeg = _finite(pitch_deg, "pitch_deg")
        self.RollDeg = _finite(roll_deg, "roll_deg")
        self.SourceName = str(source_name)
        self._time_source = time_source
        self._start_time = _finite(self._time_source(), "time_source result")
        self._last_time = self._start_time
        self._sequence_number = 0

        direction = 1.0 if self.Clockwise else -1.0
        self._angular_rate_rad_per_sec = direction * self.SpeedMps / self.RadiusM

    @property
    def OrbitPeriodSec(self) -> float:
        """Return the time for one circuit, or infinity when stationary."""

        if self.SpeedMps == 0.0:
            return math.inf
        return 2.0 * math.pi * self.RadiusM / self.SpeedMps

    def _now(self, current_time_sec: Optional[float] = None) -> float:
        now = _finite(
            self._time_source() if current_time_sec is None else current_time_sec,
            "current_time_sec",
        )
        if now < self._last_time:
            raise ValueError("navigation time cannot move backwards")
        self._last_time = now
        return now

    def get_pose(
        self,
        current_time_sec: Optional[float] = None,
    ) -> NavigationPose:
        """Return the analytically derived navigation pose at one epoch."""

        timestamp = self._now(current_time_sec)
        elapsed_sec = timestamp - self._start_time
        initial_angle_rad = math.radians(self.InitialRadialBearingDeg)
        radial_angle_rad = (
            initial_angle_rad
            + self._angular_rate_rad_per_sec * elapsed_sec
        )

        sin_angle = math.sin(radial_angle_rad)
        cos_angle = math.cos(radial_angle_rad)
        east_m = self.CentreEastM + self.RadiusM * sin_angle
        north_m = self.CentreNorthM + self.RadiusM * cos_angle
        east_mps = (
            self.RadiusM
            * cos_angle
            * self._angular_rate_rad_per_sec
        )
        north_mps = (
            -self.RadiusM
            * sin_angle
            * self._angular_rate_rad_per_sec
        )

        if self.SpeedMps > 0.0:
            heading_true_deg = wrap360(
                math.degrees(math.atan2(east_mps, north_mps))
            )
        else:
            tangent_offset_deg = 90.0 if self.Clockwise else -90.0
            heading_true_deg = wrap360(
                self.InitialRadialBearingDeg + tangent_offset_deg
            )

        position_enu = EnuPosition(
            east_m=east_m,
            north_m=north_m,
            up_m=self.AltitudeM,
        )
        velocity_enu = EnuVelocity(
            east_mps=east_mps,
            north_mps=north_mps,
            up_mps=0.0,
        )
        self._sequence_number += 1
        return NavigationPose(
            TimestampSec=timestamp,
            SequenceNumber=self._sequence_number,
            Geodetic=self.Frame.to_geodetic(position_enu),
            PositionEnu=position_enu,
            VelocityEnu=velocity_enu,
            HeadingTrueDeg=heading_true_deg,
            PitchDeg=self.PitchDeg,
            RollDeg=self.RollDeg,
            YawRateDegPerSec=math.degrees(
                self._angular_rate_rad_per_sec
            ),
            PositionValid=True,
            HeadingValid=True,
            VelocityValid=True,
            Source=self.SourceName,
        )

    def get_attitude(
        self,
        current_time_sec: Optional[float] = None,
    ) -> PlatformAttitude:
        """Return the same legacy attitude view as other navigation sources."""

        return self.get_pose(current_time_sec).as_attitude()
