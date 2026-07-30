"""Operational geometry gate for Vanguard X moving-platform PPI operation."""

from dataclasses import asdict, dataclass
import json
import math

from CoordinateFrames import (
    GeodeticPosition,
    angle_in_frame_to_true_bearing,
    signed_angle_difference_degrees,
)
from NavigationState import CircularRouteNavigationSource
from PointingManager import PointingManager
from RadarMapOverlay import PlatformTrajectory
from RadarTasks import AngleFrame, MakeSearchTask


class _Clock:
    def __init__(self):
        self.TimeSec = 0.0

    def __call__(self):
        return self.TimeSec


@dataclass(frozen=True)
class MovingPlatformPpiValidationResult:
    Frames: int
    OrbitPeriodSec: float
    RouteDiameterM: float
    RouteLengthM: float
    MaximumRadiusErrorM: float
    MaximumSectorBearingErrorDeg: float
    MaximumTrueScanRateErrorDegPerSec: float
    MaximumTrailEndpointErrorM: float
    TrailPoints: int
    Passed: bool


def RunMovingPlatformPpiValidation(
    frame_count: int = 145,
) -> MovingPlatformPpiValidationResult:
    if int(frame_count) < 9:
        raise ValueError("frame_count must be at least nine")

    clock = _Clock()
    navigation = CircularRouteNavigationSource(
        mission_origin=GeodeticPosition(-34.368, 150.929, 0.0),
        centre_east_m=5000.0,
        centre_north_m=0.0,
        radius_m=1000.0,
        speed_mps=5.0,
        clockwise=True,
        initial_radial_bearing_deg=0.0,
        time_source=clock,
    )
    sector = MakeSearchTask(
        TaskId=1,
        SectorStartDeg=40.0,
        SectorStopDeg=90.0,
        ScanRateDegPerSec=20.0,
        SectorFrame=AngleFrame.TRUE,
    )
    trajectory = PlatformTrajectory(
        maximum_points=4096,
        minimum_step_m=2.0,
    )

    maximum_radius_error_m = 0.0
    maximum_sector_error_deg = 0.0
    maximum_rate_error_dps = 0.0
    maximum_trail_endpoint_error_m = 0.0
    route_length_m = 0.0
    previous_position = None

    for index in range(int(frame_count)):
        clock.TimeSec = (
            navigation.OrbitPeriodSec
            * index
            / float(frame_count - 1)
        )
        pose = navigation.get_pose()
        east_m = float(pose.PositionEnu.east_m)
        north_m = float(pose.PositionEnu.north_m)
        trajectory.AddPosition(east_m, north_m)

        radius_m = math.hypot(
            east_m - navigation.CentreEastM,
            north_m - navigation.CentreNorthM,
        )
        maximum_radius_error_m = max(
            maximum_radius_error_m,
            abs(radius_m - navigation.RadiusM),
        )

        if previous_position is not None:
            route_length_m += math.hypot(
                east_m - previous_position[0],
                north_m - previous_position[1],
            )
        previous_position = (east_m, north_m)

        for true_bearing_deg in (40.0, 90.0):
            relative_deg = PointingManager.TrueToRelativeAzimuth(
                true_bearing_deg,
                pose.HeadingTrueDeg,
            )
            recovered_true_deg = PointingManager.RelativeToTrueBearing(
                relative_deg,
                pose.HeadingTrueDeg,
            )
            ppi_true_deg = angle_in_frame_to_true_bearing(
                true_bearing_deg,
                AngleFrame.TRUE,
                pose.HeadingTrueDeg,
            )
            maximum_sector_error_deg = max(
                maximum_sector_error_deg,
                abs(signed_angle_difference_degrees(
                    recovered_true_deg,
                    true_bearing_deg,
                )),
                abs(signed_angle_difference_degrees(
                    ppi_true_deg,
                    true_bearing_deg,
                )),
            )

        for direction in (-1.0, +1.0):
            relative_rate_dps = (
                PointingManager._search_relative_slew_rate(
                    sector,
                    pose,
                    direction,
                )
            )
            recovered_true_rate_dps = (
                relative_rate_dps + pose.YawRateDegPerSec
            )
            requested_true_rate_dps = direction * 20.0
            maximum_rate_error_dps = max(
                maximum_rate_error_dps,
                abs(
                    recovered_true_rate_dps
                    - requested_true_rate_dps
                ),
            )

        trail_points = trajectory.RelativePoints()
        if trail_points:
            maximum_trail_endpoint_error_m = max(
                maximum_trail_endpoint_error_m,
                math.hypot(*trail_points[-1]),
            )

    expected_route_length_m = 2.0 * math.pi * navigation.RadiusM
    passed = bool(
        maximum_radius_error_m <= 1.0e-9
        and maximum_sector_error_deg <= 1.0e-9
        and maximum_rate_error_dps <= 1.0e-9
        and maximum_trail_endpoint_error_m <= 1.0e-9
        and abs(route_length_m - expected_route_length_m)
        / expected_route_length_m
        <= 0.001
    )
    return MovingPlatformPpiValidationResult(
        Frames=int(frame_count),
        OrbitPeriodSec=float(navigation.OrbitPeriodSec),
        RouteDiameterM=2.0 * float(navigation.RadiusM),
        RouteLengthM=route_length_m,
        MaximumRadiusErrorM=maximum_radius_error_m,
        MaximumSectorBearingErrorDeg=maximum_sector_error_deg,
        MaximumTrueScanRateErrorDegPerSec=maximum_rate_error_dps,
        MaximumTrailEndpointErrorM=maximum_trail_endpoint_error_m,
        TrailPoints=len(trajectory.RelativePoints()),
        Passed=passed,
    )


def Main() -> int:
    result = RunMovingPlatformPpiValidation()
    print("Vanguard X moving-platform PPI operational validation")
    print(
        f"Route: diameter={result.RouteDiameterM:.1f} m, "
        f"length={result.RouteLengthM:.1f} m, "
        f"period={result.OrbitPeriodSec:.3f} s"
    )
    print(
        f"Frames/trail points: {result.Frames}/{result.TrailPoints}"
    )
    print(
        "Maximum errors: "
        f"radius={result.MaximumRadiusErrorM:.3e} m, "
        f"sector={result.MaximumSectorBearingErrorDeg:.3e} deg, "
        f"scan-rate={result.MaximumTrueScanRateErrorDegPerSec:.3e} deg/s, "
        f"trail-centre={result.MaximumTrailEndpointErrorM:.3e} m"
    )
    print(f"RESULT: {'PASS' if result.Passed else 'FAIL'}")
    if not result.Passed:
        print(json.dumps(asdict(result), indent=2, sort_keys=True))
    return 0 if result.Passed else 1


if __name__ == "__main__":
    raise SystemExit(Main())
