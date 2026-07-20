"""Select one equivalent RF target from TargetScenario return descriptors."""

from collections import OrderedDict
import math

import numpy as np


def SignedAngleDifferenceDeg(angle_deg, reference_deg):
    return (
        (float(angle_deg) - float(reference_deg) + 180.0) % 360.0
    ) - 180.0


def SelectStrongestScenarioTarget(
    SceneReturns,
    BoresightDeg,
    AngleHalfWidthDeg=2.0,
    MinimumRangeM=0.0,
    MaximumRangeM=float("inf"),
):
    """Return the strongest in-gate parent as one equivalent point target.

    Extended-target scatterers are grouped by ``parent_name``. Their powers
    are added for selection, and power-weighted geometry provides the single
    range, bearing and radial velocity used by the one-pulse RF emulator.
    """
    if float(AngleHalfWidthDeg) < 0.0:
        raise ValueError("AngleHalfWidthDeg must not be negative")
    if float(MaximumRangeM) < float(MinimumRangeM):
        raise ValueError("MaximumRangeM must not be below MinimumRangeM")

    groups = OrderedDict()
    for item in list(SceneReturns or []):
        try:
            range_m = float(item["range_m"])
            bearing_deg = float(item["bearing_deg"])
            velocity_mps = float(item["radial_velocity_mps"])
            amplitude = abs(float(item.get("amplitude", 1.0)))
        except (KeyError, TypeError, ValueError):
            continue

        values = np.asarray(
            [range_m, bearing_deg, velocity_mps, amplitude],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            continue
        if not float(MinimumRangeM) <= range_m <= float(MaximumRangeM):
            continue

        angle_error_deg = SignedAngleDifferenceDeg(
            bearing_deg,
            BoresightDeg,
        )
        if abs(angle_error_deg) > float(AngleHalfWidthDeg):
            continue

        parent_name = str(item.get("parent_name", item.get("name", "Target")))
        power_weight = amplitude * amplitude
        if power_weight <= 0.0:
            continue

        group = groups.setdefault(parent_name, [])
        group.append({
            "RangeM": range_m,
            "BearingDeg": bearing_deg,
            "RadialVelocityMps": velocity_mps,
            "PowerWeight": power_weight,
        })

    if not groups:
        return None

    candidates = []
    for parent_name, members in groups.items():
        weights = np.asarray(
            [member["PowerWeight"] for member in members],
            dtype=float,
        )
        total_power = float(np.sum(weights))
        ranges = np.asarray(
            [member["RangeM"] for member in members],
            dtype=float,
        )
        velocities = np.asarray(
            [member["RadialVelocityMps"] for member in members],
            dtype=float,
        )
        bearings_rad = np.deg2rad([
            member["BearingDeg"] for member in members
        ])
        bearing_deg = math.degrees(math.atan2(
            float(np.sum(weights * np.sin(bearings_rad))),
            float(np.sum(weights * np.cos(bearings_rad))),
        ))

        candidates.append({
            "Name": parent_name,
            "RangeM": float(np.sum(weights * ranges) / total_power),
            "BearingDeg": float(bearing_deg),
            "RadialVelocityMps": float(
                np.sum(weights * velocities) / total_power
            ),
            "AngleErrorDeg": float(
                SignedAngleDifferenceDeg(bearing_deg, BoresightDeg)
            ),
            "EquivalentAmplitude": float(math.sqrt(total_power)),
            "StrengthPower": total_power,
            "ConstituentReturnCount": len(members),
        })

    return max(candidates, key=lambda candidate: candidate["StrengthPower"])
