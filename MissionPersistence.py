"""Persistent last-known-good Mission draft handling.

The saved profile is only an editor convenience.  It is revalidated against
the current system limits at startup and is never loaded or started for
execution automatically.
"""

from __future__ import annotations

import json
import os
import tempfile
from typing import Any, Dict, Optional

from MissionProfile import (
    MissionProfile,
    MissionProfileFromDict,
    MissionProfileToDict,
    ReconcileMissionWaveforms,
)
from MissionValidator import MissionValidator


def LastKnownGoodMissionPath(config: Dict[str, Any]) -> str:
    configured = str(config.get("MissionLastKnownGoodPath", "")).strip()
    if configured:
        return os.path.abspath(os.path.expanduser(configured))
    return os.path.join(
        os.path.expanduser("~"),
        ".config",
        "VanguardX",
        "last_known_good_mission.vxmission.json",
    )


def LoadLastKnownGoodMission(
    config: Dict[str, Any],
    validator: MissionValidator,
) -> Optional[MissionProfile]:
    """Return the saved Mission only if it remains valid for this runtime."""

    path = LastKnownGoodMissionPath(config)
    try:
        with open(path, "r", encoding="utf-8") as stream:
            profile = MissionProfileFromDict(json.load(stream))
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return None

    profile = ReconcileMissionWaveforms(
        profile,
        validator.Limits.AvailableWaveformIds,
    )
    if not validator.Validate(profile).IsValid:
        return None
    return profile


def SaveLastKnownGoodMission(
    profile: MissionProfile,
    config: Dict[str, Any],
) -> str:
    """Atomically save a Mission draft that has already passed validation."""

    path = LastKnownGoodMissionPath(config)
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    temporary_path = ""
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=directory,
            prefix=".last_known_good_mission.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = stream.name
            json.dump(
                MissionProfileToDict(profile),
                stream,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    finally:
        if temporary_path and os.path.exists(temporary_path):
            os.unlink(temporary_path)
    return path
