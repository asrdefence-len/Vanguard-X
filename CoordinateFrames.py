"""Coordinate transforms for the Vanguard X navigation and radar system.

Conventions
-----------
* Geographic coordinates use the WGS-84 ellipsoid.
* Local Cartesian coordinates use ENU: East, North, Up, in metres.
* True bearing is degrees clockwise from true north.
* X6-60 encoder angle is degrees clockwise when viewed from above.
* The Vanguard X X6-60 encoder zero points aft, 180 degrees from the bow.

This module contains geometry only.  It deliberately has no dependencies on
the tracker, graphical display, hardware drivers, or simulation sources.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


_WGS84_SEMI_MAJOR_AXIS_M = 6_378_137.0
_WGS84_FLATTENING = 1.0 / 298.257_223_563
_WGS84_ECCENTRICITY_SQUARED = (
    _WGS84_FLATTENING * (2.0 - _WGS84_FLATTENING)
)


def normalise_bearing_degrees(angle_degrees: float) -> float:
    """Return an angle in the half-open interval [0, 360)."""

    result = float(angle_degrees) % 360.0
    # Avoid exposing 360 degrees because of floating-point round-off.
    return 0.0 if math.isclose(result, 360.0, abs_tol=1e-12) else result


def signed_angle_difference_degrees(
    angle_degrees: float,
    reference_degrees: float,
) -> float:
    """Return angle-reference in the interval [-180, 180)."""

    return (
        normalise_bearing_degrees(angle_degrees - reference_degrees + 180.0)
        - 180.0
    )


@dataclass(frozen=True)
class GeodeticPosition:
    """A WGS-84 latitude, longitude, and ellipsoidal altitude."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.latitude_deg):
            raise ValueError("latitude_deg must be finite")
        if not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError("latitude_deg must be between -90 and 90 degrees")
        if not math.isfinite(self.longitude_deg):
            raise ValueError("longitude_deg must be finite")
        if not math.isfinite(self.altitude_m):
            raise ValueError("altitude_m must be finite")


@dataclass(frozen=True)
class EnuPosition:
    """A position in a mission-local East-North-Up frame, in metres."""

    east_m: float
    north_m: float
    up_m: float = 0.0


@dataclass(frozen=True)
class EnuVelocity:
    """Earth-referenced velocity components in metres per second."""

    east_mps: float
    north_mps: float
    up_mps: float = 0.0


@dataclass(frozen=True)
class RelativePolarPosition:
    """Radar-relative range and true bearing."""

    range_m: float
    true_bearing_deg: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.range_m) or self.range_m < 0.0:
            raise ValueError("range_m must be finite and non-negative")
        if not math.isfinite(self.true_bearing_deg):
            raise ValueError("true_bearing_deg must be finite")
        object.__setattr__(
            self,
            "true_bearing_deg",
            normalise_bearing_degrees(self.true_bearing_deg),
        )


def _geodetic_to_ecef(position: GeodeticPosition) -> Tuple[float, float, float]:
    latitude_rad = math.radians(position.latitude_deg)
    longitude_rad = math.radians(position.longitude_deg)
    sin_latitude = math.sin(latitude_rad)
    cos_latitude = math.cos(latitude_rad)
    sin_longitude = math.sin(longitude_rad)
    cos_longitude = math.cos(longitude_rad)

    prime_vertical_radius_m = _WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        1.0 - _WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
    )

    x_m = (
        prime_vertical_radius_m + position.altitude_m
    ) * cos_latitude * cos_longitude
    y_m = (
        prime_vertical_radius_m + position.altitude_m
    ) * cos_latitude * sin_longitude
    z_m = (
        prime_vertical_radius_m
        * (1.0 - _WGS84_ECCENTRICITY_SQUARED)
        + position.altitude_m
    ) * sin_latitude
    return x_m, y_m, z_m


def _ecef_to_geodetic(
    x_m: float,
    y_m: float,
    z_m: float,
) -> GeodeticPosition:
    longitude_rad = math.atan2(y_m, x_m)
    horizontal_radius_m = math.hypot(x_m, y_m)

    if horizontal_radius_m < 1e-9:
        latitude_deg = 90.0 if z_m >= 0.0 else -90.0
        polar_radius_m = _WGS84_SEMI_MAJOR_AXIS_M * (
            1.0 - _WGS84_FLATTENING
        )
        return GeodeticPosition(
            latitude_deg=latitude_deg,
            longitude_deg=0.0,
            altitude_m=abs(z_m) - polar_radius_m,
        )

    latitude_rad = math.atan2(
        z_m,
        horizontal_radius_m * (1.0 - _WGS84_ECCENTRICITY_SQUARED),
    )

    altitude_m = 0.0
    for _ in range(10):
        sin_latitude = math.sin(latitude_rad)
        prime_vertical_radius_m = _WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
            1.0 - _WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
        )
        cos_latitude = math.cos(latitude_rad)
        if abs(cos_latitude) < 1e-15:
            altitude_m = abs(z_m) - (
                prime_vertical_radius_m
                * (1.0 - _WGS84_ECCENTRICITY_SQUARED)
            )
        else:
            altitude_m = horizontal_radius_m / cos_latitude - (
                prime_vertical_radius_m
            )

        denominator = horizontal_radius_m * (
            1.0
            - (
                _WGS84_ECCENTRICITY_SQUARED
                * prime_vertical_radius_m
                / (prime_vertical_radius_m + altitude_m)
            )
        )
        next_latitude_rad = math.atan2(z_m, denominator)
        if abs(next_latitude_rad - latitude_rad) < 1e-14:
            latitude_rad = next_latitude_rad
            break
        latitude_rad = next_latitude_rad

    sin_latitude = math.sin(latitude_rad)
    prime_vertical_radius_m = _WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        1.0 - _WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
    )
    cos_latitude = math.cos(latitude_rad)
    altitude_m = (
        horizontal_radius_m / cos_latitude - prime_vertical_radius_m
        if abs(cos_latitude) >= 1e-15
        else abs(z_m)
        - (
            prime_vertical_radius_m
            * (1.0 - _WGS84_ECCENTRICITY_SQUARED)
        )
    )

    return GeodeticPosition(
        latitude_deg=math.degrees(latitude_rad),
        longitude_deg=normalise_longitude_degrees(math.degrees(longitude_rad)),
        altitude_m=altitude_m,
    )


def normalise_longitude_degrees(longitude_degrees: float) -> float:
    """Return longitude in the interval [-180, 180)."""

    return (float(longitude_degrees) + 180.0) % 360.0 - 180.0


class LocalEnuFrame:
    """A mission-local ENU frame anchored at a fixed WGS-84 origin."""

    def __init__(self, origin: GeodeticPosition):
        self.origin = origin
        self._origin_ecef_m = _geodetic_to_ecef(origin)

        latitude_rad = math.radians(origin.latitude_deg)
        longitude_rad = math.radians(origin.longitude_deg)
        self._sin_latitude = math.sin(latitude_rad)
        self._cos_latitude = math.cos(latitude_rad)
        self._sin_longitude = math.sin(longitude_rad)
        self._cos_longitude = math.cos(longitude_rad)

    def to_enu(self, position: GeodeticPosition) -> EnuPosition:
        """Convert a WGS-84 position into this mission-local ENU frame."""

        x_m, y_m, z_m = _geodetic_to_ecef(position)
        delta_x_m = x_m - self._origin_ecef_m[0]
        delta_y_m = y_m - self._origin_ecef_m[1]
        delta_z_m = z_m - self._origin_ecef_m[2]

        east_m = (
            -self._sin_longitude * delta_x_m
            + self._cos_longitude * delta_y_m
        )
        north_m = (
            -self._sin_latitude * self._cos_longitude * delta_x_m
            - self._sin_latitude * self._sin_longitude * delta_y_m
            + self._cos_latitude * delta_z_m
        )
        up_m = (
            self._cos_latitude * self._cos_longitude * delta_x_m
            + self._cos_latitude * self._sin_longitude * delta_y_m
            + self._sin_latitude * delta_z_m
        )
        return EnuPosition(east_m=east_m, north_m=north_m, up_m=up_m)

    def to_geodetic(self, position: EnuPosition) -> GeodeticPosition:
        """Convert a mission-local ENU position back to WGS-84."""

        delta_x_m = (
            -self._sin_longitude * position.east_m
            - self._sin_latitude
            * self._cos_longitude
            * position.north_m
            + self._cos_latitude
            * self._cos_longitude
            * position.up_m
        )
        delta_y_m = (
            self._cos_longitude * position.east_m
            - self._sin_latitude
            * self._sin_longitude
            * position.north_m
            + self._cos_latitude
            * self._sin_longitude
            * position.up_m
        )
        delta_z_m = (
            self._cos_latitude * position.north_m
            + self._sin_latitude * position.up_m
        )

        return _ecef_to_geodetic(
            self._origin_ecef_m[0] + delta_x_m,
            self._origin_ecef_m[1] + delta_y_m,
            self._origin_ecef_m[2] + delta_z_m,
        )


def relative_polar_to_enu_delta(
    range_m: float,
    true_bearing_deg: float,
) -> EnuPosition:
    """Convert radar-relative range/true-bearing into an ENU displacement."""

    if not math.isfinite(range_m) or range_m < 0.0:
        raise ValueError("range_m must be finite and non-negative")
    if not math.isfinite(true_bearing_deg):
        raise ValueError("true_bearing_deg must be finite")

    bearing_rad = math.radians(true_bearing_deg)
    return EnuPosition(
        east_m=range_m * math.sin(bearing_rad),
        north_m=range_m * math.cos(bearing_rad),
        up_m=0.0,
    )


def enu_delta_to_relative_polar(
    east_m: float,
    north_m: float,
) -> RelativePolarPosition:
    """Convert an ENU displacement into radar-relative range/true-bearing."""

    if not math.isfinite(east_m) or not math.isfinite(north_m):
        raise ValueError("east_m and north_m must be finite")

    range_m = math.hypot(east_m, north_m)
    bearing_deg = (
        0.0
        if range_m == 0.0
        else normalise_bearing_degrees(math.degrees(math.atan2(east_m, north_m)))
    )
    return RelativePolarPosition(
        range_m=range_m,
        true_bearing_deg=bearing_deg,
    )


def detection_to_enu_position(
    radar_position: EnuPosition,
    measured_range_m: float,
    true_bearing_deg: float,
) -> EnuPosition:
    """Place a radar detection into the mission ENU frame."""

    offset = relative_polar_to_enu_delta(
        range_m=measured_range_m,
        true_bearing_deg=true_bearing_deg,
    )
    return EnuPosition(
        east_m=radar_position.east_m + offset.east_m,
        north_m=radar_position.north_m + offset.north_m,
        up_m=radar_position.up_m + offset.up_m,
    )


def target_relative_to_radar(
    target_position: EnuPosition,
    radar_position: EnuPosition,
) -> RelativePolarPosition:
    """Calculate current target range and true bearing from the radar."""

    return enu_delta_to_relative_polar(
        east_m=target_position.east_m - radar_position.east_m,
        north_m=target_position.north_m - radar_position.north_m,
    )


def x660_encoder_to_true_bearing(
    encoder_angle_deg: float,
    vessel_true_heading_deg: float,
    *,
    encoder_zero_from_bow_deg: float = 180.0,
    installation_correction_deg: float = 0.0,
    encoder_clockwise_positive: bool = True,
) -> float:
    """Convert X6-60 encoder angle to beam true bearing.

    The Vanguard X default is encoder zero aft, hence
    ``encoder_zero_from_bow_deg=180``.
    """

    encoder_direction = 1.0 if encoder_clockwise_positive else -1.0
    return normalise_bearing_degrees(
        vessel_true_heading_deg
        + encoder_zero_from_bow_deg
        + encoder_direction * encoder_angle_deg
        + installation_correction_deg
    )


def true_bearing_to_x660_encoder(
    true_bearing_deg: float,
    vessel_true_heading_deg: float,
    *,
    encoder_zero_from_bow_deg: float = 180.0,
    installation_correction_deg: float = 0.0,
    encoder_clockwise_positive: bool = True,
) -> float:
    """Convert a desired beam true bearing to an X6-60 encoder angle."""

    encoder_direction = 1.0 if encoder_clockwise_positive else -1.0
    clockwise_encoder_angle_deg = normalise_bearing_degrees(
        true_bearing_deg
        - vessel_true_heading_deg
        - encoder_zero_from_bow_deg
        - installation_correction_deg
    )
    return normalise_bearing_degrees(
        encoder_direction * clockwise_encoder_angle_deg
    )


def line_of_sight_velocity_mps(
    velocity: EnuVelocity,
    true_bearing_deg: float,
) -> float:
    """Project an ENU velocity onto the outward radar line of sight."""

    bearing_rad = math.radians(true_bearing_deg)
    return (
        velocity.east_mps * math.sin(bearing_rad)
        + velocity.north_mps * math.cos(bearing_rad)
    )

