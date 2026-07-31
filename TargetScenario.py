"""
===============================================================================
ASR Defence X-Band Radar Prototype
TargetScenario.py
===============================================================================

Purpose
-------
This file defines a simple 2D radar scene model for the X-band prototype.

It supports:
    - multiple scene objects / targets
    - Earth-fixed north/east target positions
    - Earth-fixed north/east target velocities
    - radar range, bearing, radial velocity and Doppler calculation
    - scanning radar boresight angles
    - sinc-squared antenna beam pattern
    - mainbeam and sidelobe returns
    - two-way antenna gain for monostatic radar operation

Important modelling note
------------------------
Objects are NOT removed just because they are outside the main beam.
Every scene object contributes to the received signal, but its amplitude is
scaled by the antenna pattern at its bearing offset from the current boresight.

This allows strong objects to appear through sidelobes.
===============================================================================
"""

import numpy as np


# -----------------------------------------------------------------------------
# Angle utilities
# -----------------------------------------------------------------------------

def wrap_angle_deg(angle_deg):
    """
    Wrap an angle to the interval [-180, +180) degrees.
    """
    return (angle_deg + 180.0) % 360.0 - 180.0


# -----------------------------------------------------------------------------
# Scene object creation
# -----------------------------------------------------------------------------

def make_scene_object_from_range_bearing(
    name,
    range_m,
    bearing_deg,
    speed_mps,
    heading_deg,
    rcs=1.0,
):
    """
    Create a scene object from polar position and heading.

    Parameters
    ----------
    name : str
        Name of the object.

    range_m : float
        Initial range from radar in metres.

    bearing_deg : float
        Initial true bearing from radar in degrees.
        0 degrees is north (+x).
        90 degrees is east (+y).

    speed_mps : float
        Object speed in metres per second.

    heading_deg : float
        Direction of motion in degrees true.
        0 degrees is north (+x).
        90 degrees is east (+y).
        180 degrees is south (-x).

    rcs : float
        Relative radar cross section. This is a simulation scaling value.
    """

    bearing_rad = np.deg2rad(bearing_deg)
    heading_rad = np.deg2rad(heading_deg)

    return {
        "name": name,
        "x_m": range_m * np.cos(bearing_rad),
        "y_m": range_m * np.sin(bearing_rad),
        "vx_mps": speed_mps * np.cos(heading_rad),
        "vy_mps": speed_mps * np.sin(heading_rad),
        "rcs": rcs,
    }


def make_extended_line_target_from_range_bearing(
    name,
    range_m,
    bearing_deg,
    speed_mps,
    heading_deg,
    length_m=200.0,
    aspect_deg=None,
    num_scatterers=21,
    total_rcs=5000.0,
    rcs_taper="cosine",
):
    """
    Create a simple extended target as a line of point scatterers.

    This is useful for simulating a container vessel / large ship with a
    physically long range profile. The returned object is expanded into many
    scatterer returns by build_scene_returns_for_boresight().

    Parameters
    ----------
    length_m : float
        Physical target length in metres.

    aspect_deg : float or None
        Orientation of the vessel long axis in world coordinates. If None,
        the heading direction is used. If the long axis points approximately
        toward/away from the radar, the target spreads mainly in range.

    num_scatterers : int
        Number of discrete scatterers used along the target length.

    total_rcs : float
        Total target RCS spread across all scatterers.
    """

    bearing_rad = np.deg2rad(bearing_deg)
    heading_rad = np.deg2rad(heading_deg)

    if aspect_deg is None:
        aspect_deg = heading_deg
    aspect_rad = np.deg2rad(aspect_deg)

    return {
        "name": name,
        "x_m": range_m * np.cos(bearing_rad),
        "y_m": range_m * np.sin(bearing_rad),
        "vx_mps": speed_mps * np.cos(heading_rad),
        "vy_mps": speed_mps * np.sin(heading_rad),
        "rcs": total_rcs,
        "extended": True,
        "model": "line",
        "length_m": float(length_m),
        "aspect_deg": float(aspect_deg),
        "num_scatterers": int(num_scatterers),
        "total_rcs": float(total_rcs),
        "rcs_taper": str(rcs_taper).lower(),
    }



def create_default_scene():
    """
    Create the moving-platform operational simulation scene.

    The simulated radar follows a 1 km-radius circle centred 5 km east of the
    mission origin.  These target tracks are deliberately placed outside that
    route so every scatterer remains at least 4 km from every possible platform
    position for the first 20 minutes of the scenario.  This prevents minimum-
    range blanking and very-near-target CFAR contamination from dominating the
    normal moving-platform validation.

    The moving vessels head generally away from the platform route.  The
    stationary reflector remains between their initial bearings so the scene
    still exercises strong-target and sidelobe behaviour.

    Returns
    -------
    scene_objects : list of dict
        List of scene objects.
    """

    scene_objects = [
        # A 200 m container-vessel-like target.  Its long axis remains close to
        # the initial line of sight to retain the intended extended range
        # profile without crossing the platform route.
        make_extended_line_target_from_range_bearing(
            name="Container_Vessel_200m",
            range_m=7000.0,
            bearing_deg=30.0,
            speed_mps=5.0,
            heading_deg=30.0,
            length_m=200.0,
            aspect_deg=30.0,
            num_scatterers=25,
            total_rcs=40000.0,
            rcs_taper="cosine",
        ),
        make_extended_line_target_from_range_bearing(
            name="Container_Vessel1_200m",
            range_m=10000.0,
            bearing_deg=110.0,
            speed_mps=5.0,
            heading_deg=110.0,
            length_m=150.0,
            aspect_deg=110.0,
            num_scatterers=25,
            total_rcs=12000.0,
            rcs_taper="cosine",
        ),
        make_scene_object_from_range_bearing(
            name="Reflector_-35deg",
            range_m=11000.0,
            bearing_deg=70.0,
            speed_mps=0.0,
            heading_deg=0.0,
            rcs=800.0,
        ),
    ]

    return scene_objects


# -----------------------------------------------------------------------------
# Scene update
# -----------------------------------------------------------------------------

def update_scene_objects(scene_objects, delta_time_s):
    """
    Move all scene objects forward in time using constant velocity.
    """

    for obj in scene_objects:
        obj["x_m"] += obj["vx_mps"] * delta_time_s
        obj["y_m"] += obj["vy_mps"] * delta_time_s


# -----------------------------------------------------------------------------
# Geometry and Doppler
# -----------------------------------------------------------------------------

def calculate_object_geometry(obj, radar_params):
    """
    Calculate range, bearing, radial velocity and Doppler for one object.

    Parameters
    ----------
    obj : dict
        Scene object containing x_m, y_m, vx_mps, vy_mps.

    radar_params : dict
        Must include carrier_frequency_hz.
        Can include radar_x_m and radar_y_m. Defaults to 0,0.
        Can also include radar_vx_mps and radar_vy_mps. These are required for
        physically correct Doppler when the radar platform is moving.

        Legacy x/y convention:
            x = mission north
            y = mission east

        This convention is retained so existing scenes remain compatible.

    Returns
    -------
    geometry : dict
        Range, bearing, radial velocity and Doppler.
    """

    c = 3.0e8
    wavelength_m = c / radar_params["carrier_frequency_hz"]

    radar_x = radar_params.get("radar_x_m", 0.0)
    radar_y = radar_params.get("radar_y_m", 0.0)
    radar_vx = radar_params.get("radar_vx_mps", 0.0)
    radar_vy = radar_params.get("radar_vy_mps", 0.0)

    dx = obj["x_m"] - radar_x
    dy = obj["y_m"] - radar_y

    range_m = np.sqrt(dx**2 + dy**2)

    if range_m <= 0.0:
        range_m = 1.0

    bearing_deg = np.rad2deg(np.arctan2(dy, dx))

    los_x = dx / range_m
    los_y = dy / range_m

    relative_vx_mps = obj["vx_mps"] - radar_vx
    relative_vy_mps = obj["vy_mps"] - radar_vy
    # This is the Doppler-bearing radial velocity the radar actually measures,
    # not the target's Earth-referenced LOS velocity. Positive is outward /
    # receding. Ownship velocity is subtracted exactly once here.
    measured_relative_radial_velocity_mps = (
        relative_vx_mps * los_x
        + relative_vy_mps * los_y
    )

    # Monostatic radar Doppler.
    doppler_hz = (
        2.0
        * measured_relative_radial_velocity_mps
        / wavelength_m
    )

    return {
        "range_m": range_m,
        "bearing_deg": bearing_deg,
        # Retain radial_velocity_mps for SimulatedSource compatibility. Its
        # meaning is explicitly the measured relative radial velocity.
        "radial_velocity_mps": measured_relative_radial_velocity_mps,
        "measured_relative_radial_velocity_mps": (
            measured_relative_radial_velocity_mps
        ),
        "doppler_hz": doppler_hz,
        "radar_los_velocity_mps": (
            radar_vx * los_x + radar_vy * los_y
        ),
        "target_los_velocity_mps": (
            obj["vx_mps"] * los_x + obj["vy_mps"] * los_y
        ),
    }


# -----------------------------------------------------------------------------
# Antenna beam model
# -----------------------------------------------------------------------------

def antenna_gain_power_sinc(angle_error_deg, beamwidth_deg, sidelobe_floor_db=-50.0):
    """
    Sinc-squared one-way antenna power pattern.

    The first null occurs approximately at +/- beamwidth_deg.

    This is not a perfect physical aperture model, but it is very useful for
    showing:
        - a main beam
        - sidelobes
        - large targets leaking through sidelobes

    Parameters
    ----------
    angle_error_deg : float
        Target bearing minus radar boresight, in degrees.

    beamwidth_deg : float
        Approximate angle to first null, in degrees.
        If set to 5 deg, the first null is at about +/- 5 deg.

    sidelobe_floor_db : float
        Numerical floor for the one-way power gain in dB.
        This prevents perfect nulls from becoming exactly zero.

    Returns
    -------
    gain_power : float
        One-way antenna power gain, normalised to 1.0 at boresight.
    """

    if beamwidth_deg <= 0.0:
        raise ValueError("beamwidth_deg must be greater than zero")

    x = angle_error_deg / beamwidth_deg

    gain_power = np.sinc(x) ** 2

    floor_linear = 10.0 ** (sidelobe_floor_db / 10.0)
    gain_power = np.maximum(gain_power, floor_linear)

    return gain_power


# -----------------------------------------------------------------------------
# Scan schedules
# -----------------------------------------------------------------------------

def create_scan_angles(start_deg=0.0, stop_deg=90.0, step_deg=1.0):
    """
    Create a simple one-way scan from start_deg to stop_deg.
    """

    return np.arange(start_deg, stop_deg + step_deg, step_deg)



def create_ping_pong_scan_angles(start_deg=0.0, stop_deg=90.0, step_deg=1.0):
    """
    Create a back-and-forth scan.

    Example:
        0, 1, 2, ..., 90, 89, 88, ..., 1
    """

    forward_scan = np.arange(start_deg, stop_deg + step_deg, step_deg)
    reverse_scan = np.arange(stop_deg - step_deg, start_deg, -step_deg)

    return np.concatenate((forward_scan, reverse_scan))


# -----------------------------------------------------------------------------
# Scene return builder
# -----------------------------------------------------------------------------

def build_scene_returns_for_boresight(scene_objects, radar_params):
    """
    Convert all scene objects into radar return descriptors for one dwell angle.

    All scene objects are included. Their amplitudes are scaled by:
        - RCS
        - range loss
        - two-way antenna gain

    Parameters
    ----------
    scene_objects : list of dict
        Objects in the scene.

    radar_params : dict
        Must include:
            carrier_frequency_hz
            boresight_deg
            beamwidth_deg

        Optional:
            radar_x_m
            radar_y_m
            radar_vx_mps
            radar_vy_mps
            reference_range_m
            target_amplitude_scale
            sidelobe_floor_db

    Returns
    -------
    scene_returns : list of dict
        Return descriptors for the receiver simulator.
    """

    boresight_deg = radar_params["boresight_deg"]
    beamwidth_deg = radar_params["beamwidth_deg"]

    reference_range_m = radar_params.get("reference_range_m", 8000.0)
    target_amplitude_scale = radar_params.get("target_amplitude_scale", 1.0)
    sidelobe_floor_db = radar_params.get("sidelobe_floor_db", -50.0)

    scene_returns = []

    for obj in scene_objects:
        objects_to_render = [obj]

        # Expand a line-style extended target into point scatterers.
        if bool(obj.get("extended", False)) and obj.get("model", "line") == "line":
            num_scatterers = max(1, int(obj.get("num_scatterers", 21)))
            length_m = float(obj.get("length_m", 200.0))
            aspect_rad = np.deg2rad(float(obj.get("aspect_deg", 0.0)))
            total_rcs = float(obj.get("total_rcs", obj.get("rcs", 1.0)))
            taper = str(obj.get("rcs_taper", "cosine")).lower()

            offsets_m = np.linspace(-0.5 * length_m, 0.5 * length_m, num_scatterers)

            if taper == "cosine" and num_scatterers > 1:
                weights = 0.35 + 0.65 * np.hanning(num_scatterers)
            elif taper == "ends":
                weights = np.ones(num_scatterers)
                weights[0] *= 2.0
                weights[-1] *= 2.0
            else:
                weights = np.ones(num_scatterers)

            weights = weights / np.sum(weights)

            objects_to_render = []
            for scatterer_index, offset_m in enumerate(offsets_m):
                objects_to_render.append(
                    {
                        "name": f"{obj['name']}_S{scatterer_index + 1:02d}",
                        "x_m": obj["x_m"] + offset_m * np.cos(aspect_rad),
                        "y_m": obj["y_m"] + offset_m * np.sin(aspect_rad),
                        "vx_mps": obj["vx_mps"],
                        "vy_mps": obj["vy_mps"],
                        "rcs": total_rcs * float(weights[scatterer_index]),
                        "parent_name": obj["name"],
                        "extended_parent": True,
                        "scatterer_index": scatterer_index,
                        "scatterer_count": num_scatterers,
                        "scatterer_offset_m": float(offset_m),
                    }
                )

        for render_obj in objects_to_render:
            geometry = calculate_object_geometry(render_obj, radar_params)

            angle_error_deg = wrap_angle_deg(
                geometry["bearing_deg"] - boresight_deg
            )

            one_way_gain_power = antenna_gain_power_sinc(
                angle_error_deg=angle_error_deg,
                beamwidth_deg=beamwidth_deg,
                sidelobe_floor_db=sidelobe_floor_db,
            )

            # Monostatic radar: same antenna on transmit and receive.
            # Power is scaled by Gtx * Grx, so for same antenna it is G^2.
            two_way_gain_power = one_way_gain_power ** 2

            # Voltage/amplitude scales as sqrt(power).
            # Radar power range loss is roughly 1/R^4, so amplitude is roughly 1/R^2.
            amplitude = (
                target_amplitude_scale
                * np.sqrt(render_obj["rcs"])
                * np.sqrt(two_way_gain_power)
                * (reference_range_m / geometry["range_m"]) ** 2
            )

            if two_way_gain_power > 0.0:
                two_way_gain_db = 10.0 * np.log10(two_way_gain_power)
            else:
                two_way_gain_db = -300.0

            scene_returns.append(
                {
                    "name": render_obj["name"],
                    "parent_name": render_obj.get("parent_name", render_obj["name"]),
                    "extended_parent": bool(render_obj.get("extended_parent", False)),
                    "scatterer_index": render_obj.get("scatterer_index", None),
                    "scatterer_count": render_obj.get("scatterer_count", None),
                    "scatterer_offset_m": render_obj.get("scatterer_offset_m", 0.0),
                    "range_m": geometry["range_m"],
                    "bearing_deg": geometry["bearing_deg"],
                    "angle_error_deg": angle_error_deg,
                    "radial_velocity_mps": geometry["radial_velocity_mps"],
                    "doppler_hz": geometry["doppler_hz"],
                    "rcs": render_obj["rcs"],
                    "one_way_gain_power": one_way_gain_power,
                    "two_way_gain_power": two_way_gain_power,
                    "two_way_gain_db": two_way_gain_db,
                    "amplitude": amplitude,
                }
            )

    return scene_returns


# -----------------------------------------------------------------------------
# Debug printing
# -----------------------------------------------------------------------------

def print_scene_returns(scene_returns, boresight_deg):
    """
    Print a useful truth table for the current dwell.
    """

    print("")
    print(f"Radar boresight angle: {boresight_deg:.1f} deg")
    print("")
    print("Scene returns through antenna pattern:")
    print(
        f"{'Name':32s} "
        f"{'Range(m)':>10s} "
        f"{'Bearing':>9s} "
        f"{'Err':>9s} "
        f"{'Doppler':>10s} "
        f"{'2wayGain':>10s} "
        f"{'Amp':>10s}"
    )
    print("-" * 100)

    for ret in scene_returns:
        print(
            f"{ret['name']:32s} "
            f"{ret['range_m']:10.1f} "
            f"{ret['bearing_deg']:8.2f}d "
            f"{ret['angle_error_deg']:8.2f}d "
            f"{ret['doppler_hz']:9.1f}Hz "
            f"{ret['two_way_gain_db']:9.1f}dB "
            f"{ret['amplitude']:10.4e}"
        )
