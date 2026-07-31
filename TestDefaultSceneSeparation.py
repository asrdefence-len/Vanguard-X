"""Operational geometry gate for the default moving-platform target scene."""

import math
import unittest

import numpy as np

from TargetScenario import create_default_scene


class TestDefaultSceneSeparation(unittest.TestCase):
    PLATFORM_CIRCLE_CENTRE_EAST_M = 5000.0
    PLATFORM_CIRCLE_CENTRE_NORTH_M = 0.0
    PLATFORM_CIRCLE_RADIUS_M = 1000.0
    VALIDATION_DURATION_S = 20.0 * 60.0
    VALIDATION_STEP_S = 10.0
    MINIMUM_SCATTERER_RANGE_M = 4000.0
    MAXIMUM_SCATTERER_RANGE_M = 15000.0

    @staticmethod
    def _scatterer_offsets(scene_object):
        if not bool(scene_object.get("extended", False)):
            return ((0.0, 0.0),)

        count = max(1, int(scene_object.get("num_scatterers", 1)))
        length_m = float(scene_object.get("length_m", 0.0))
        aspect_rad = math.radians(
            float(scene_object.get("aspect_deg", 0.0))
        )
        return tuple(
            (
                float(offset_m) * math.cos(aspect_rad),
                float(offset_m) * math.sin(aspect_rad),
            )
            for offset_m in np.linspace(
                -0.5 * length_m,
                0.5 * length_m,
                count,
            )
        )

    def test_all_scatterers_remain_in_operational_range_annulus(self):
        """Check the worst case over every possible point on the orbit."""

        minimum_range_m = math.inf
        maximum_range_m = 0.0
        minimum_identity = None

        times_s = np.arange(
            0.0,
            self.VALIDATION_DURATION_S + self.VALIDATION_STEP_S,
            self.VALIDATION_STEP_S,
        )
        for scene_object in create_default_scene():
            for time_s in times_s:
                object_north_m = (
                    float(scene_object["x_m"])
                    + float(scene_object["vx_mps"]) * float(time_s)
                )
                object_east_m = (
                    float(scene_object["y_m"])
                    + float(scene_object["vy_mps"]) * float(time_s)
                )

                for scatterer_index, (
                    offset_north_m,
                    offset_east_m,
                ) in enumerate(self._scatterer_offsets(scene_object)):
                    relative_east_m = (
                        object_east_m
                        + offset_east_m
                        - self.PLATFORM_CIRCLE_CENTRE_EAST_M
                    )
                    relative_north_m = (
                        object_north_m
                        + offset_north_m
                        - self.PLATFORM_CIRCLE_CENTRE_NORTH_M
                    )
                    centre_range_m = math.hypot(
                        relative_east_m,
                        relative_north_m,
                    )
                    closest_orbit_range_m = (
                        centre_range_m - self.PLATFORM_CIRCLE_RADIUS_M
                    )
                    farthest_orbit_range_m = (
                        centre_range_m + self.PLATFORM_CIRCLE_RADIUS_M
                    )

                    if closest_orbit_range_m < minimum_range_m:
                        minimum_range_m = closest_orbit_range_m
                        minimum_identity = (
                            scene_object["name"],
                            scatterer_index,
                            float(time_s),
                        )
                    maximum_range_m = max(
                        maximum_range_m,
                        farthest_orbit_range_m,
                    )

        self.assertGreaterEqual(
            minimum_range_m,
            self.MINIMUM_SCATTERER_RANGE_M,
            msg=(
                f"closest return {minimum_identity} is only "
                f"{minimum_range_m:.1f} m from the platform orbit"
            ),
        )
        self.assertLessEqual(
            maximum_range_m,
            self.MAXIMUM_SCATTERER_RANGE_M,
            msg=(
                f"a default-scene return can reach "
                f"{maximum_range_m:.1f} m from the platform orbit"
            ),
        )

        print(
            "Default scene separation: "
            f"minimum={minimum_range_m / 1000.0:.3f} km, "
            f"maximum={maximum_range_m / 1000.0:.3f} km, "
            f"duration={self.VALIDATION_DURATION_S / 60.0:.0f} min"
        )


if __name__ == "__main__":
    unittest.main()
