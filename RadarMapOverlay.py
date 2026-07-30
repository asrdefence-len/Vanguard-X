"""Offline radar-centred vector map support for the Vanguard X PPI."""

from collections import deque
import json
import math
import os


EARTH_RADIUS_M = 6371008.8


class PlatformTrajectory:
    """Bounded Earth-fixed breadcrumb history projected about the live radar.

    Stored points use mission East/North coordinates.  Projection subtracts
    the latest platform position, so the current radar is always at ``(0, 0)``
    while its travelled path moves beneath the north-up PPI.
    """

    def __init__(self, maximum_points=4096, minimum_step_m=2.0):
        maximum_points = int(maximum_points)
        minimum_step_m = float(minimum_step_m)
        if maximum_points < 2:
            raise ValueError("maximum_points must be at least two")
        if not math.isfinite(minimum_step_m) or minimum_step_m < 0.0:
            raise ValueError("minimum_step_m must be finite and non-negative")

        self.MaximumPoints = maximum_points
        self.MinimumStepM = minimum_step_m
        self._history = deque(maxlen=maximum_points)
        self._current = None

    def AddPosition(self, east_m, north_m):
        east_m = float(east_m)
        north_m = float(north_m)
        if not math.isfinite(east_m) or not math.isfinite(north_m):
            raise ValueError("platform position must be finite")

        self._current = (east_m, north_m)
        if not self._history:
            self._history.append(self._current)
            return True

        last_east_m, last_north_m = self._history[-1]
        if (
            math.hypot(
                east_m - last_east_m,
                north_m - last_north_m,
            )
            < self.MinimumStepM
        ):
            return False

        self._history.append(self._current)
        return True

    def RelativePoints(self):
        """Return trail points relative to the current radar position."""

        if self._current is None:
            return []

        current_east_m, current_north_m = self._current
        points = list(self._history)
        if not points or points[-1] != self._current:
            points.append(self._current)
        return [
            (
                east_m - current_east_m,
                north_m - current_north_m,
            )
            for east_m, north_m in points
        ]

    def Clear(self):
        self._history.clear()
        self._current = None

    def __len__(self):
        return len(self._history)


def LocalEastNorthM(latitude_deg, longitude_deg, centre_latitude_deg, centre_longitude_deg):
    """Convert WGS-84 latitude/longitude to local east/north metres."""
    mean_latitude_rad = math.radians(
        0.5 * (float(latitude_deg) + float(centre_latitude_deg))
    )
    east_m = EARTH_RADIUS_M * math.radians(
        float(longitude_deg) - float(centre_longitude_deg)
    ) * math.cos(mean_latitude_rad)
    north_m = EARTH_RADIUS_M * math.radians(
        float(latitude_deg) - float(centre_latitude_deg)
    )
    return east_m, north_m


def LoadRadarMapDataset(filename):
    """Load and minimally validate a Vanguard X offline map JSON file."""
    with open(filename, "r", encoding="utf-8") as map_file:
        dataset = json.load(map_file)

    coastlines = dataset.get("coastlines")
    if coastlines is None:
        coastlines = [dataset.get("coastline", [])]
    if not coastlines or any(len(line) < 2 for line in coastlines):
        raise ValueError("Radar map dataset must contain valid coastline geometry")
    for line in coastlines:
        for point in line:
            if not isinstance(point, list) or len(point) != 2:
                raise ValueError("Each coastline point must be [longitude_deg, latitude_deg]")
            if not all(math.isfinite(float(value)) for value in point):
                raise ValueError("Radar map coastline contains a non-finite coordinate")

    return dataset


class RadarCentredMap:
    """Project a cached regional vector dataset about the current radar fix."""

    def __init__(self, dataset_filename, latitude_deg, longitude_deg):
        self.DatasetFilename = os.path.abspath(dataset_filename)
        self.Dataset = LoadRadarMapDataset(self.DatasetFilename)
        self.SetRadarPosition(latitude_deg, longitude_deg)

    def SetRadarPosition(self, latitude_deg, longitude_deg):
        latitude_deg = float(latitude_deg)
        longitude_deg = float(longitude_deg)
        if not math.isfinite(latitude_deg) or not -90.0 <= latitude_deg <= 90.0:
            raise ValueError("Radar latitude must be finite and within -90..90 degrees")
        if not math.isfinite(longitude_deg) or not -180.0 <= longitude_deg <= 180.0:
            raise ValueError("Radar longitude must be finite and within -180..180 degrees")
        self.LatitudeDeg = latitude_deg
        self.LongitudeDeg = longitude_deg

    def ProjectCoordinate(self, longitude_deg, latitude_deg):
        return LocalEastNorthM(
            latitude_deg,
            longitude_deg,
            self.LatitudeDeg,
            self.LongitudeDeg,
        )

    def ProjectCoastline(self):
        """Legacy single-coastline API used by Bellambi schema version 1."""
        return self.ProjectCoastlines()[0]

    def ProjectCoastlines(self):
        lines = self.Dataset.get("coastlines")
        if lines is None:
            lines = [self.Dataset["coastline"]]
        return [
            [self.ProjectCoordinate(*point) for point in line]
            for line in lines
        ]

    def ProjectLandPolygons(self):
        polygons = self.Dataset.get("land_polygons")
        if polygons is not None:
            return [
                [self.ProjectCoordinate(*point) for point in polygon]
                for polygon in polygons
            ]
        coastline = self.ProjectCoastline()
        west_m = min(point[0] for point in coastline) - 50000.0
        return [coastline + [
            (west_m, coastline[-1][1]),
            (west_m, coastline[0][1]),
        ]]

    def ProjectLabels(self, maximum_range_m):
        labels = []
        maximum_range_m = float(maximum_range_m)
        for record in self.Dataset.get("labels", []):
            east_m, north_m = self.ProjectCoordinate(
                record["longitude_deg"], record["latitude_deg"]
            )
            if math.hypot(east_m, north_m) <= maximum_range_m:
                labels.append((str(record["name"]), east_m, north_m))
        return labels
