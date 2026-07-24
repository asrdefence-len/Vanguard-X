"""Draft/validated/loaded mission ownership for Vanguard X.

This controller intentionally stops at LOADED.  Scheduler execution is the
next staged increment; no method in this class commands radar or motion
hardware.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Optional

from MissionProfile import CloneMission, MissionProfile
from MissionValidator import MissionValidationResult, MissionValidator


class MissionController:
    def __init__(self, validator: MissionValidator, initial_profile: MissionProfile):
        self.Validator = validator
        self.DraftProfile = CloneMission(initial_profile)
        self.ValidatedProfile: Optional[MissionProfile] = None
        self.ValidationResult: Optional[MissionValidationResult] = None
        self.LoadedProfile: Optional[MissionProfile] = None
        self.State = "STOPPED"

    def SetDraft(self, profile: MissionProfile):
        self.DraftProfile = CloneMission(profile)
        self.ValidatedProfile = None
        self.ValidationResult = None

    def ValidateDraft(self) -> MissionValidationResult:
        result = self.Validator.Validate(self.DraftProfile)
        self.ValidationResult = result
        if result.IsValid:
            self.ValidatedProfile = replace(
                CloneMission(self.DraftProfile),
                ValidationSnapshot=result.Snapshot(),
            )
        else:
            self.ValidatedProfile = None
        return result

    def LoadValidated(self) -> MissionProfile:
        if self.ValidatedProfile is None or self.ValidationResult is None:
            raise RuntimeError("mission draft has not passed validation")
        if not self.ValidationResult.IsValid:
            raise RuntimeError("mission draft contains validation errors")
        self.LoadedProfile = CloneMission(self.ValidatedProfile)
        self.State = "LOADED"
        return CloneMission(self.LoadedProfile)

    def ClearLoaded(self):
        self.LoadedProfile = None
        self.State = "STOPPED"
