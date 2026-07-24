"""Qt Mission Setup page for Vanguard X.

The page edits mission intent and emits versioned operator requests only.  It
has no Ettus, scheduler, antenna, ATR, or hardware imports.  Authoritative
mission state is returned by the radar-side MissionExecutionController.
"""

from __future__ import annotations

from dataclasses import replace
import json
import os
from typing import Optional

from PyQt5 import QtCore, QtGui, QtWidgets

from MissionController import MissionController
from MissionPersistence import (
    LoadLastKnownGoodMission,
    SaveLastKnownGoodMission,
)
from MissionProfile import (
    CloneMission,
    CreateDefaultMission,
    MissionProfileFromDict,
    MissionProfileToDict,
    NewTaskId,
    ReconcileMissionWaveforms,
    ResolveAvailableWaveformId,
    Scan360Task,
    SectorScanTask,
    TouchMission,
    TrackUpdatePolicy,
)
from MissionValidator import MissionValidator


class MissionPage(QtWidgets.QWidget):
    statusChanged = QtCore.pyqtSignal()

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.Config = dict(config)
        self.Validator = MissionValidator.FromConfig(self.Config)
        initial_profile = LoadLastKnownGoodMission(
            self.Config,
            self.Validator,
        )
        if initial_profile is None:
            initial_profile = ReconcileMissionWaveforms(
                CreateDefaultMission(),
                self.Validator.Limits.AvailableWaveformIds,
            )
        self.Controller = MissionController(
            self.Validator,
            initial_profile,
        )
        self.SelectedTaskIndex = 0
        self._LoadingWidgets = False
        self.MissionCommandRevision = 0
        self.MissionCommand = ""
        self.MissionCommandProfile = None
        self.RuntimeStatus = {
            "State": "STOPPED",
            "Message": "No mission loaded",
            "StatusRevision": 0,
        }
        self._BuildUi()
        self._LoadMissionWidgets()
        self._RefreshTaskTable()
        self._LoadSelectedTaskEditor()
        self._RefreshValidation(None)

    @property
    def DraftProfile(self):
        return self.Controller.DraftProfile

    @property
    def LoadedProfile(self):
        return self.Controller.LoadedProfile

    def GetMissionControlState(self):
        """Return the latest edge-triggered mission request for the radar."""

        return {
            "MissionCommandRevision": int(self.MissionCommandRevision),
            "MissionCommand": str(self.MissionCommand),
            "MissionProfile": self.MissionCommandProfile,
        }

    def SetRuntimeStatus(self, status):
        """Apply authoritative radar-side mission state to the page."""

        self.RuntimeStatus = dict(status or {})
        state = str(self.RuntimeStatus.get("State", "STOPPED")).upper()
        self.StartButton.setText("Start Mission")
        self.StartButton.setEnabled(state == "LOADED")
        self.PauseButton.setText("Resume" if state == "PAUSED" else "Pause")
        self.PauseButton.setEnabled(
            state in ("RUNNING_360", "RUNNING_SECTOR", "PAUSED")
        )
        self.LoadButton.setEnabled(
            bool(
                self.Controller.ValidationResult is not None
                and self.Controller.ValidationResult.IsValid
            )
            and state not in ("RUNNING_360", "RUNNING_SECTOR", "PAUSED")
        )
        self._UpdateStateLabel()
        self.statusChanged.emit()

    def RequestStopFromPersistentStop(self):
        """Couple the always-visible STOP button to mission abort."""

        state = str(self.RuntimeStatus.get("State", "STOPPED")).upper()
        if state in (
            "LOADED", "RUNNING_360", "RUNNING_SECTOR", "PAUSED",
        ):
            self._QueueMissionCommand("STOP")

    def _QueueMissionCommand(self, command, profile=None):
        self.MissionCommandRevision += 1
        self.MissionCommand = str(command).upper()
        self.MissionCommandProfile = (
            None if profile is None else MissionProfileToDict(profile)
        )
        self.RuntimeStatus = {
            **self.RuntimeStatus,
            "Message": f"{self.MissionCommand} requested",
        }
        self._UpdateStateLabel()
        self.statusChanged.emit()

    def MissionStatusText(self):
        runtime_state = str(self.RuntimeStatus.get("State", "STOPPED"))
        runtime_name = str(self.RuntimeStatus.get("MissionName", "")).strip()
        task_name = str(self.RuntimeStatus.get("ActiveTaskName", "")).strip()
        if runtime_state.startswith("RUNNING"):
            running_text = (
                f"{runtime_state}: {task_name}"
                if task_name else runtime_state
            )
            waveform = str(
                self.RuntimeStatus.get("ActiveWaveformId", "")
            ).strip()
            prf_hz = self.RuntimeStatus.get("ActivePrfHz")
            pulses = self.RuntimeStatus.get("ActivePulsesPerCpi")
            if waveform:
                timing_parts = [waveform]
                if prf_hz is not None:
                    timing_parts.append(f"{float(prf_hz) / 1000.0:g} kHz")
                if pulses is not None:
                    timing_parts.append(f"{int(pulses)} pulses")
                running_text += " | " + ", ".join(timing_parts)
            next_name = str(
                self.RuntimeStatus.get("NextPeriodicTaskName", "")
            ).strip()
            due_sec = self.RuntimeStatus.get("NextPeriodicDueInSec")
            if next_name and due_sec is not None:
                running_text += (
                    f" | {next_name} in {float(due_sec) / 60.0:.1f} min"
                )
            return running_text
        if runtime_state == "PAUSED":
            return f"PAUSED: {task_name}" if task_name else "PAUSED"
        if runtime_state == "LOADED":
            revision = int(self.RuntimeStatus.get("MissionRevision", 0))
            return f"LOADED: {runtime_name} r{revision}"
        if runtime_state in ("COMPLETED", "ABORTED", "FAULTED"):
            return runtime_state
        loaded = self.Controller.LoadedProfile
        if loaded is not None:
            return f"LOAD REQUESTED: {loaded.Name} r{loaded.Revision}"
        result = self.Controller.ValidationResult
        if result is not None and result.IsValid:
            return f"VALIDATED DRAFT: {self.DraftProfile.Name}"
        return f"DRAFT: {self.DraftProfile.Name}"

    def _BuildUi(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        identity_box = QtWidgets.QGroupBox("Mission identity")
        identity = QtWidgets.QGridLayout(identity_box)
        self.NameEdit = QtWidgets.QLineEdit()
        self.IdEdit = QtWidgets.QLineEdit()
        self.RevisionSpin = QtWidgets.QSpinBox()
        self.RevisionSpin.setRange(1, 999999)
        self.CompletionCombo = QtWidgets.QComboBox()
        self.CompletionCombo.addItems(["STOP", "REPEAT"])
        self.DescriptionEdit = QtWidgets.QLineEdit()
        identity.addWidget(QtWidgets.QLabel("Name"), 0, 0)
        identity.addWidget(self.NameEdit, 0, 1)
        identity.addWidget(QtWidgets.QLabel("Mission ID"), 0, 2)
        identity.addWidget(self.IdEdit, 0, 3)
        identity.addWidget(QtWidgets.QLabel("Revision"), 0, 4)
        identity.addWidget(self.RevisionSpin, 0, 5)
        identity.addWidget(QtWidgets.QLabel("Completion"), 0, 6)
        identity.addWidget(self.CompletionCombo, 0, 7)
        identity.addWidget(QtWidgets.QLabel("Description"), 1, 0)
        identity.addWidget(self.DescriptionEdit, 1, 1, 1, 7)
        outer.addWidget(identity_box)

        body = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        outer.addWidget(body, stretch=1)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.TaskTable = QtWidgets.QTableWidget(0, 6)
        self.TaskTable.setHorizontalHeaderLabels(
            ["Order", "Enabled", "Task", "Duration", "Summary", "Validation"]
        )
        self.TaskTable.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows
        )
        self.TaskTable.setSelectionMode(
            QtWidgets.QAbstractItemView.SingleSelection
        )
        self.TaskTable.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.TaskTable.verticalHeader().setVisible(False)
        header = self.TaskTable.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(5, QtWidgets.QHeaderView.ResizeToContents)
        self.TaskTable.itemSelectionChanged.connect(self._OnTaskSelectionChanged)
        left_layout.addWidget(self.TaskTable, stretch=1)

        task_buttons = QtWidgets.QHBoxLayout()
        for label, slot in (
            ("+ 360", self._Add360),
            ("+ Sector", self._AddSector),
            ("Up", lambda: self._MoveTask(-1)),
            ("Down", lambda: self._MoveTask(1)),
            ("Duplicate", self._DuplicateTask),
            ("Remove", self._RemoveTask),
        ):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(slot)
            task_buttons.addWidget(button)
        task_buttons.addStretch(1)
        left_layout.addLayout(task_buttons)
        body.addWidget(left)

        right_scroll = QtWidgets.QScrollArea()
        right_scroll.setWidgetResizable(True)
        right = QtWidgets.QWidget()
        right_scroll.setWidget(right)
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(4, 0, 4, 0)

        self.TaskEditorBox = QtWidgets.QGroupBox("Selected primary task")
        task_form = QtWidgets.QGridLayout(self.TaskEditorBox)
        self.TaskEnabledCheck = QtWidgets.QCheckBox("Enabled")
        self.TaskNameEdit = QtWidgets.QLineEdit()
        self.TaskContinuousCheck = QtWidgets.QCheckBox("Continuous")
        self.TaskDurationMinutes = self._DoubleSpin(0.1, 100000.0, 10.0, " min")
        self.TaskPeriodicCheck = QtWidgets.QCheckBox("Periodic interrupt")
        self.TaskRepeatMinutes = self._DoubleSpin(
            0.1, 100000.0, 1.0, " min"
        )
        self.TaskPeriodicCheck.setToolTip(
            "Pre-empts the continuous baseline scan at this interval."
        )
        self.TaskTypeLabel = QtWidgets.QLabel()
        self.TaskWaveformCombo = QtWidgets.QComboBox()
        self.TaskWaveformCombo.addItems(
            [str(item) for item in self.Validator.Limits.AvailableWaveformIds]
        )
        self.TaskPrfKHz = self._DoubleSpin(
            self.Validator.Limits.MinimumPrfHz / 1000.0,
            self.Validator.Limits.MaximumPrfHz / 1000.0,
            0.1,
            " kHz",
            decimals=2,
        )
        self.TaskPulses = QtWidgets.QSpinBox()
        self.TaskPulses.setRange(
            self.Validator.Limits.MinimumPulsesPerCpi,
            self.Validator.Limits.MaximumPulsesPerCpi,
        )
        self.TaskRangeKm = self._DoubleSpin(
            self.Validator.Limits.MinimumRangeM / 1000.0,
            self.Validator.Limits.MaximumRangeM / 1000.0,
            0.5,
            " km",
            decimals=1,
        )
        task_form.addWidget(self.TaskTypeLabel, 0, 0, 1, 2)
        task_form.addWidget(self.TaskEnabledCheck, 0, 2)
        task_form.addWidget(QtWidgets.QLabel("Name"), 1, 0)
        task_form.addWidget(self.TaskNameEdit, 1, 1, 1, 2)
        task_form.addWidget(QtWidgets.QLabel("Duration"), 2, 0)
        task_form.addWidget(self.TaskDurationMinutes, 2, 1)
        task_form.addWidget(self.TaskContinuousCheck, 2, 2)
        task_form.addWidget(QtWidgets.QLabel("Repeat"), 3, 0)
        task_form.addWidget(self.TaskRepeatMinutes, 3, 1)
        task_form.addWidget(self.TaskPeriodicCheck, 3, 2)
        task_form.addWidget(QtWidgets.QLabel("Waveform"), 4, 0)
        task_form.addWidget(self.TaskWaveformCombo, 4, 1, 1, 2)
        task_form.addWidget(QtWidgets.QLabel("PRF"), 5, 0)
        task_form.addWidget(self.TaskPrfKHz, 5, 1)
        task_form.addWidget(QtWidgets.QLabel("Pulses/CPI"), 6, 0)
        task_form.addWidget(self.TaskPulses, 6, 1)
        task_form.addWidget(QtWidgets.QLabel("Maximum range"), 7, 0)
        task_form.addWidget(self.TaskRangeKm, 7, 1)

        self.MotionStack = QtWidgets.QStackedWidget()
        task_form.addWidget(self.MotionStack, 8, 0, 1, 3)
        self.MotionStack.addWidget(self._Build360Editor())
        self.MotionStack.addWidget(self._BuildSectorEditor())
        right_layout.addWidget(self.TaskEditorBox)

        right_layout.addWidget(self._BuildTrackUpdateEditor())
        right_layout.addStretch(1)
        body.addWidget(right_scroll)
        body.setStretchFactor(0, 3)
        body.setStretchFactor(1, 2)

        results = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.ValidationTable = QtWidgets.QTableWidget(0, 3)
        self.ValidationTable.setHorizontalHeaderLabels(
            ["Severity", "Field", "Message"]
        )
        self.ValidationTable.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.ValidationTable.verticalHeader().setVisible(False)
        self.ValidationTable.horizontalHeader().setSectionResizeMode(
            0, QtWidgets.QHeaderView.ResizeToContents
        )
        self.ValidationTable.horizontalHeader().setSectionResizeMode(
            1, QtWidgets.QHeaderView.ResizeToContents
        )
        self.ValidationTable.horizontalHeader().setSectionResizeMode(
            2, QtWidgets.QHeaderView.Stretch
        )
        self.DerivedText = QtWidgets.QPlainTextEdit()
        self.DerivedText.setReadOnly(True)
        self.DerivedText.setMaximumBlockCount(500)
        results.addWidget(self.ValidationTable)
        results.addWidget(self.DerivedText)
        results.setStretchFactor(0, 3)
        results.setStretchFactor(1, 2)
        results.setMinimumHeight(190)
        outer.addWidget(results)

        actions = QtWidgets.QHBoxLayout()
        self.NewButton = QtWidgets.QPushButton("New")
        self.OpenButton = QtWidgets.QPushButton("Load file")
        self.SaveButton = QtWidgets.QPushButton("Save file")
        self.ValidateButton = QtWidgets.QPushButton("Validate Mission")
        self.LoadButton = QtWidgets.QPushButton("Load Mission")
        self.StartButton = QtWidgets.QPushButton("Start Mission")
        self.PauseButton = QtWidgets.QPushButton("Pause")
        self.StartButton.setEnabled(False)
        self.PauseButton.setEnabled(False)
        self.StartButton.setToolTip(
            "Starts the immutable loaded snapshot in SIM."
        )
        self.PauseButton.setToolTip(
            "Pauses or resumes at a safe dwell boundary."
        )
        for button in (
            self.NewButton, self.OpenButton, self.SaveButton,
            self.ValidateButton, self.LoadButton,
            self.StartButton, self.PauseButton,
        ):
            actions.addWidget(button)
        actions.addStretch(1)
        self.StateLabel = QtWidgets.QLabel()
        self.StateLabel.setStyleSheet("font-weight: bold;")
        actions.addWidget(self.StateLabel)
        outer.addLayout(actions)

        self.NewButton.clicked.connect(self._NewMission)
        self.OpenButton.clicked.connect(self._OpenMission)
        self.SaveButton.clicked.connect(self._SaveMission)
        self.ValidateButton.clicked.connect(self._ValidateMission)
        self.LoadButton.clicked.connect(self._LoadValidatedMission)
        self.StartButton.clicked.connect(self._StartMission)
        self.PauseButton.clicked.connect(self._PauseOrResumeMission)

        for widget in (
            self.NameEdit,
            self.IdEdit,
            self.DescriptionEdit,
        ):
            widget.editingFinished.connect(self._OnMissionIdentityChanged)
        self.RevisionSpin.valueChanged.connect(self._OnMissionIdentityChanged)
        self.CompletionCombo.currentTextChanged.connect(
            self._OnMissionIdentityChanged
        )

        self._ConnectTaskEditors()
        self._ConnectTrackEditors()

    def _DoubleSpin(
        self, minimum, maximum, step, suffix="", decimals=2,
    ):
        widget = QtWidgets.QDoubleSpinBox()
        widget.setRange(float(minimum), float(maximum))
        widget.setSingleStep(float(step))
        widget.setDecimals(int(decimals))
        widget.setSuffix(suffix)
        return widget

    def _Build360Editor(self):
        widget = QtWidgets.QGroupBox("360 Scan motion")
        layout = QtWidgets.QFormLayout(widget)
        self.Scan360Direction = QtWidgets.QComboBox()
        self.Scan360Direction.addItems(["CW", "CCW"])
        self.Scan360Rpm = self._DoubleSpin(0.1, 120.0, 0.5, " RPM")
        layout.addRow("Direction", self.Scan360Direction)
        layout.addRow("Commanded rotation", self.Scan360Rpm)
        return widget

    def _BuildSectorEditor(self):
        widget = QtWidgets.QGroupBox("Sector Scan motion")
        layout = QtWidgets.QFormLayout(widget)
        self.SectorStartDeg = self._DoubleSpin(0.0, 359.999, 1.0, " deg", 2)
        self.SectorStopDeg = self._DoubleSpin(0.0, 359.999, 1.0, " deg", 2)
        self.SectorDirection = QtWidgets.QComboBox()
        self.SectorDirection.addItems(["CW", "CCW"])
        self.SectorRate = self._DoubleSpin(0.1, 720.0, 1.0, " deg/s")
        self.SectorMargin = self._DoubleSpin(0.0, 90.0, 0.25, " deg")
        layout.addRow("Start bearing", self.SectorStartDeg)
        layout.addRow("Stop bearing", self.SectorStopDeg)
        layout.addRow("Initial direction", self.SectorDirection)
        layout.addRow("Scan rate", self.SectorRate)
        layout.addRow("Endpoint margin", self.SectorMargin)
        return widget

    def _BuildTrackUpdateEditor(self):
        box = QtWidgets.QGroupBox("Track Update interrupt policy")
        layout = QtWidgets.QGridLayout(box)
        self.TrackEnabled = QtWidgets.QCheckBox("Enabled for tagged confirmed tracks")
        self.TrackRevisit = self._DoubleSpin(0.1, 3600.0, 1.0, " s")
        self.TrackWaveform = QtWidgets.QComboBox()
        self.TrackWaveform.addItems(
            [str(item) for item in self.Validator.Limits.AvailableWaveformIds]
        )
        self.TrackPrfKHz = self._DoubleSpin(
            self.Validator.Limits.MinimumPrfHz / 1000.0,
            self.Validator.Limits.MaximumPrfHz / 1000.0,
            0.1,
            " kHz",
        )
        self.TrackPulses = QtWidgets.QSpinBox()
        self.TrackPulses.setRange(
            self.Validator.Limits.MinimumPulsesPerCpi,
            self.Validator.Limits.MaximumPulsesPerCpi,
        )
        self.TrackRangeKm = self._DoubleSpin(
            self.Validator.Limits.MinimumRangeM / 1000.0,
            self.Validator.Limits.MaximumRangeM / 1000.0,
            0.5,
            " km",
            1,
        )
        self.TrackGateHalf = self._DoubleSpin(0.1, 45.0, 0.25, " deg")
        self.TrackStep = self._DoubleSpin(0.05, 45.0, 0.25, " deg")
        self.TrackDwells = QtWidgets.QSpinBox()
        self.TrackDwells.setRange(1, 32)
        self.TrackSlewRate = self._DoubleSpin(0.1, 720.0, 1.0, " deg/s")
        self.TrackNodRate = self._DoubleSpin(0.1, 720.0, 1.0, " deg/s")
        self.TrackTolerance = self._DoubleSpin(0.01, 20.0, 0.05, " deg")
        self.TrackSettle = self._DoubleSpin(0.0, 30.0, 0.05, " s")
        self.TrackTimeout = self._DoubleSpin(0.1, 600.0, 1.0, " s")
        self.TrackMiss = QtWidgets.QComboBox()
        self.TrackMiss.addItems([
            "RETRY_WIDER_THEN_DEFER", "RETRY", "WIDEN", "DEFER",
        ])

        layout.addWidget(self.TrackEnabled, 0, 0, 1, 4)
        rows = (
            ("Revisit interval", self.TrackRevisit, "Waveform", self.TrackWaveform),
            ("PRF", self.TrackPrfKHz, "Pulses/CPI", self.TrackPulses),
            ("Maximum range", self.TrackRangeKm, "Gate half-width", self.TrackGateHalf),
            ("Angular step", self.TrackStep, "Dwells/angle", self.TrackDwells),
            ("Slew rate", self.TrackSlewRate, "Nod rate", self.TrackNodRate),
            ("Pointing tolerance", self.TrackTolerance, "Settle/angle", self.TrackSettle),
            ("Maximum duration", self.TrackTimeout, "Miss policy", self.TrackMiss),
        )
        for row, (label1, widget1, label2, widget2) in enumerate(rows, start=1):
            layout.addWidget(QtWidgets.QLabel(label1), row, 0)
            layout.addWidget(widget1, row, 1)
            layout.addWidget(QtWidgets.QLabel(label2), row, 2)
            layout.addWidget(widget2, row, 3)
        return box

    def _ConnectTaskEditors(self):
        for signal in (
            self.TaskEnabledCheck.stateChanged,
            self.TaskNameEdit.editingFinished,
            self.TaskContinuousCheck.stateChanged,
            self.TaskDurationMinutes.valueChanged,
            self.TaskPeriodicCheck.stateChanged,
            self.TaskRepeatMinutes.valueChanged,
            self.TaskWaveformCombo.currentTextChanged,
            self.TaskPrfKHz.valueChanged,
            self.TaskPulses.valueChanged,
            self.TaskRangeKm.valueChanged,
            self.Scan360Direction.currentTextChanged,
            self.Scan360Rpm.valueChanged,
            self.SectorStartDeg.valueChanged,
            self.SectorStopDeg.valueChanged,
            self.SectorDirection.currentTextChanged,
            self.SectorRate.valueChanged,
            self.SectorMargin.valueChanged,
        ):
            signal.connect(self._OnTaskEditorChanged)
        self.TaskContinuousCheck.stateChanged.connect(
            self._UpdateTaskScheduleControls
        )
        self.TaskPeriodicCheck.stateChanged.connect(
            self._UpdateTaskScheduleControls
        )

    def _UpdateTaskScheduleControls(self, *unused):
        continuous = self.TaskContinuousCheck.isChecked()
        periodic = self.TaskPeriodicCheck.isChecked()
        self.TaskDurationMinutes.setEnabled(not continuous)
        self.TaskPeriodicCheck.setEnabled(not continuous)
        self.TaskRepeatMinutes.setEnabled(not continuous and periodic)

    def _ConnectTrackEditors(self):
        for signal in (
            self.TrackEnabled.stateChanged,
            self.TrackRevisit.valueChanged,
            self.TrackWaveform.currentTextChanged,
            self.TrackPrfKHz.valueChanged,
            self.TrackPulses.valueChanged,
            self.TrackRangeKm.valueChanged,
            self.TrackGateHalf.valueChanged,
            self.TrackStep.valueChanged,
            self.TrackDwells.valueChanged,
            self.TrackSlewRate.valueChanged,
            self.TrackNodRate.valueChanged,
            self.TrackTolerance.valueChanged,
            self.TrackSettle.valueChanged,
            self.TrackTimeout.valueChanged,
            self.TrackMiss.currentTextChanged,
        ):
            signal.connect(self._OnTrackEditorChanged)

    def _LoadMissionWidgets(self):
        self._LoadingWidgets = True
        profile = self.DraftProfile
        self.NameEdit.setText(profile.Name)
        self.IdEdit.setText(profile.MissionId)
        self.RevisionSpin.setValue(profile.Revision)
        self.DescriptionEdit.setText(profile.Description)
        self.CompletionCombo.setCurrentText(profile.CompletionPolicy)
        policy = profile.TrackUpdate
        self.TrackEnabled.setChecked(policy.Enabled)
        self.TrackRevisit.setValue(policy.RevisitIntervalSec)
        self.TrackWaveform.setCurrentText(policy.WaveformId)
        self.TrackPrfKHz.setValue(policy.PrfHz / 1000.0)
        self.TrackPulses.setValue(policy.PulsesPerCpi)
        self.TrackRangeKm.setValue(policy.MaximumRangeM / 1000.0)
        self.TrackGateHalf.setValue(policy.GateHalfWidthDeg)
        self.TrackStep.setValue(policy.AngularStepDeg)
        self.TrackDwells.setValue(policy.DwellsPerAngle)
        self.TrackSlewRate.setValue(policy.SlewRateDegSec)
        self.TrackNodRate.setValue(policy.NodRateDegSec)
        self.TrackTolerance.setValue(policy.PointingToleranceDeg)
        self.TrackSettle.setValue(policy.SettleTimeSec)
        self.TrackTimeout.setValue(policy.MaximumUpdateDurationSec)
        self.TrackMiss.setCurrentText(policy.MissPolicy)
        self._LoadingWidgets = False
        self._UpdateStateLabel()

    def _LoadSelectedTaskEditor(self):
        tasks = self.DraftProfile.PrimaryTasks
        if not tasks:
            self.TaskEditorBox.setEnabled(False)
            return
        self.TaskEditorBox.setEnabled(True)
        self.SelectedTaskIndex = min(self.SelectedTaskIndex, len(tasks) - 1)
        task = tasks[self.SelectedTaskIndex]
        self._LoadingWidgets = True
        self.TaskEnabledCheck.setChecked(task.Enabled)
        self.TaskNameEdit.setText(task.Name)
        continuous = task.DurationSec is None
        self.TaskContinuousCheck.setChecked(continuous)
        if task.DurationSec is not None:
            self.TaskDurationMinutes.setValue(task.DurationSec / 60.0)
        periodic = task.RepeatIntervalSec is not None
        self.TaskPeriodicCheck.setChecked(periodic)
        if task.RepeatIntervalSec is not None:
            self.TaskRepeatMinutes.setValue(task.RepeatIntervalSec / 60.0)
        self._UpdateTaskScheduleControls()
        self.TaskWaveformCombo.setCurrentText(task.WaveformId)
        self.TaskPrfKHz.setValue(task.PrfHz / 1000.0)
        self.TaskPulses.setValue(task.PulsesPerCpi)
        self.TaskRangeKm.setValue(task.MaximumRangeM / 1000.0)
        if isinstance(task, Scan360Task):
            self.TaskTypeLabel.setText(
                f"360 SCAN  |  {task.TaskId}  |  example settings are not approved drive limits"
            )
            self.MotionStack.setCurrentIndex(0)
            self.Scan360Direction.setCurrentText(task.Direction)
            self.Scan360Rpm.setValue(task.RotationRpm)
        else:
            self.TaskTypeLabel.setText(
                f"SECTOR SCAN  |  {task.TaskId}  |  circular bearings"
            )
            self.MotionStack.setCurrentIndex(1)
            self.SectorStartDeg.setValue(task.StartDeg)
            self.SectorStopDeg.setValue(task.StopDeg)
            self.SectorDirection.setCurrentText(task.InitialDirection)
            self.SectorRate.setValue(task.ScanRateDegSec)
            self.SectorMargin.setValue(task.EndpointMarginDeg)
        self._LoadingWidgets = False

    def _OnMissionIdentityChanged(self, *unused):
        if self._LoadingWidgets:
            return
        profile = TouchMission(
            self.DraftProfile,
            Name=self.NameEdit.text().strip(),
            MissionId=self.IdEdit.text().strip(),
            Revision=self.RevisionSpin.value(),
            Description=self.DescriptionEdit.text().strip(),
            CompletionPolicy=self.CompletionCombo.currentText(),
        )
        self.Controller.SetDraft(profile)
        self._DraftChanged()

    def _OnTaskEditorChanged(self, *unused):
        if self._LoadingWidgets:
            return
        tasks = list(self.DraftProfile.PrimaryTasks)
        if not tasks or self.SelectedTaskIndex >= len(tasks):
            return
        old = tasks[self.SelectedTaskIndex]
        common = dict(
            Name=self.TaskNameEdit.text().strip(),
            Enabled=self.TaskEnabledCheck.isChecked(),
            DurationSec=(
                None if self.TaskContinuousCheck.isChecked()
                else self.TaskDurationMinutes.value() * 60.0
            ),
            RepeatIntervalSec=(
                self.TaskRepeatMinutes.value() * 60.0
                if (
                    not self.TaskContinuousCheck.isChecked()
                    and self.TaskPeriodicCheck.isChecked()
                )
                else None
            ),
            WaveformId=self.TaskWaveformCombo.currentText(),
            PrfHz=self.TaskPrfKHz.value() * 1000.0,
            PulsesPerCpi=self.TaskPulses.value(),
            MaximumRangeM=self.TaskRangeKm.value() * 1000.0,
        )
        if isinstance(old, Scan360Task):
            tasks[self.SelectedTaskIndex] = replace(
                old,
                **common,
                Direction=self.Scan360Direction.currentText(),
                RotationRpm=self.Scan360Rpm.value(),
            )
        else:
            tasks[self.SelectedTaskIndex] = replace(
                old,
                **common,
                StartDeg=self.SectorStartDeg.value(),
                StopDeg=self.SectorStopDeg.value(),
                InitialDirection=self.SectorDirection.currentText(),
                ScanRateDegSec=self.SectorRate.value(),
                EndpointMarginDeg=self.SectorMargin.value(),
            )
        self.Controller.SetDraft(TouchMission(
            self.DraftProfile,
            PrimaryTasks=tuple(tasks),
        ))
        self._DraftChanged(refresh_editor=False)

    def _OnTrackEditorChanged(self, *unused):
        if self._LoadingWidgets:
            return
        old = self.DraftProfile.TrackUpdate
        policy = replace(
            old,
            Enabled=self.TrackEnabled.isChecked(),
            RevisitIntervalSec=self.TrackRevisit.value(),
            WaveformId=self.TrackWaveform.currentText(),
            PrfHz=self.TrackPrfKHz.value() * 1000.0,
            PulsesPerCpi=self.TrackPulses.value(),
            MaximumRangeM=self.TrackRangeKm.value() * 1000.0,
            GateHalfWidthDeg=self.TrackGateHalf.value(),
            AngularStepDeg=self.TrackStep.value(),
            DwellsPerAngle=self.TrackDwells.value(),
            SlewRateDegSec=self.TrackSlewRate.value(),
            NodRateDegSec=self.TrackNodRate.value(),
            PointingToleranceDeg=self.TrackTolerance.value(),
            SettleTimeSec=self.TrackSettle.value(),
            MaximumUpdateDurationSec=self.TrackTimeout.value(),
            MissPolicy=self.TrackMiss.currentText(),
        )
        self.Controller.SetDraft(TouchMission(
            self.DraftProfile,
            TrackUpdate=policy,
        ))
        self._DraftChanged(refresh_editor=False)

    def _DraftChanged(self, refresh_editor=True):
        self._RefreshTaskTable()
        if refresh_editor:
            self._LoadSelectedTaskEditor()
        self._RefreshValidation(None)
        self._UpdateStateLabel()
        self.statusChanged.emit()

    def _RefreshTaskTable(self):
        tasks = self.DraftProfile.PrimaryTasks
        selected = self.SelectedTaskIndex
        validation = self.Controller.ValidationResult
        self.TaskTable.blockSignals(True)
        self.TaskTable.setRowCount(len(tasks))
        for row, task in enumerate(tasks):
            duration = (
                "Continuous" if task.DurationSec is None
                else f"{task.DurationSec / 60.0:g} min"
            )
            recurrence = (
                ""
                if task.RepeatIntervalSec is None
                else f"; every {task.RepeatIntervalSec / 60.0:g} min"
            )
            if isinstance(task, Scan360Task):
                task_name = "360 Scan"
                summary = (
                    f"{task.Direction}; {task.RotationRpm:g} RPM; "
                    f"{task.WaveformId}; {task.PrfHz / 1000.0:g} kHz; "
                    f"{task.PulsesPerCpi} pulses{recurrence}"
                )
            else:
                task_name = "Sector Scan"
                summary = (
                    f"{task.StartDeg:03.0f}-{task.StopDeg:03.0f} deg "
                    f"{task.InitialDirection}; {task.ScanRateDegSec:g} deg/s; "
                    f"{task.WaveformId}{recurrence}"
                )
            validation_text = "Draft"
            if validation is not None:
                task_prefix = f"PrimaryTasks[{row}]"
                task_issues = [
                    issue for issue in validation.Issues
                    if issue.Field.startswith(task_prefix)
                    or issue.Field == task_prefix
                ]
                if any(issue.Severity == "ERROR" for issue in task_issues):
                    validation_text = "Error"
                elif any(issue.Severity == "WARNING" for issue in task_issues):
                    validation_text = "Warning"
                else:
                    validation_text = "OK"
            values = [
                str(row + 1),
                "Yes" if task.Enabled else "No",
                task_name,
                duration,
                summary,
                validation_text,
            ]
            for column, value in enumerate(values):
                self.TaskTable.setItem(row, column, QtWidgets.QTableWidgetItem(value))
        self.TaskTable.blockSignals(False)
        if tasks:
            self.TaskTable.selectRow(min(selected, len(tasks) - 1))

    def _OnTaskSelectionChanged(self):
        rows = self.TaskTable.selectionModel().selectedRows()
        if not rows:
            return
        self.SelectedTaskIndex = rows[0].row()
        self._LoadSelectedTaskEditor()

    def _Add360(self):
        tasks = list(self.DraftProfile.PrimaryTasks)
        tasks.append(Scan360Task(
            TaskId=NewTaskId("scan360"),
            WaveformId=ResolveAvailableWaveformId(
                "Frank10_20MHz",
                self.Validator.Limits.AvailableWaveformIds,
            ),
        ))
        self.SelectedTaskIndex = len(tasks) - 1
        self.Controller.SetDraft(TouchMission(
            self.DraftProfile, PrimaryTasks=tuple(tasks),
        ))
        self._DraftChanged()

    def _AddSector(self):
        tasks = list(self.DraftProfile.PrimaryTasks)
        tasks.append(SectorScanTask(
            TaskId=NewTaskId("sector"),
            WaveformId=ResolveAvailableWaveformId(
                "Golay64_20MHz",
                self.Validator.Limits.AvailableWaveformIds,
            ),
        ))
        self.SelectedTaskIndex = len(tasks) - 1
        self.Controller.SetDraft(TouchMission(
            self.DraftProfile, PrimaryTasks=tuple(tasks),
        ))
        self._DraftChanged()

    def _MoveTask(self, delta):
        tasks = list(self.DraftProfile.PrimaryTasks)
        source = self.SelectedTaskIndex
        target = source + int(delta)
        if source < 0 or target < 0 or target >= len(tasks):
            return
        tasks[source], tasks[target] = tasks[target], tasks[source]
        self.SelectedTaskIndex = target
        self.Controller.SetDraft(TouchMission(
            self.DraftProfile, PrimaryTasks=tuple(tasks),
        ))
        self._DraftChanged()

    def _DuplicateTask(self):
        tasks = list(self.DraftProfile.PrimaryTasks)
        if not tasks:
            return
        source = tasks[self.SelectedTaskIndex]
        prefix = "scan360" if isinstance(source, Scan360Task) else "sector"
        duplicate = replace(
            source,
            TaskId=NewTaskId(prefix),
            Name=f"{source.Name} copy",
        )
        tasks.insert(self.SelectedTaskIndex + 1, duplicate)
        self.SelectedTaskIndex += 1
        self.Controller.SetDraft(TouchMission(
            self.DraftProfile, PrimaryTasks=tuple(tasks),
        ))
        self._DraftChanged()

    def _RemoveTask(self):
        tasks = list(self.DraftProfile.PrimaryTasks)
        if not tasks:
            return
        del tasks[self.SelectedTaskIndex]
        self.SelectedTaskIndex = max(0, self.SelectedTaskIndex - 1)
        self.Controller.SetDraft(TouchMission(
            self.DraftProfile, PrimaryTasks=tuple(tasks),
        ))
        self._DraftChanged()

    def _ValidateMission(self):
        result = self.Controller.ValidateDraft()
        if result.IsValid:
            try:
                SaveLastKnownGoodMission(
                    self.DraftProfile,
                    self.Config,
                )
            except OSError as error:
                print(
                    "Mission last-known-good save failed: "
                    f"{error}"
                )
        self._RefreshValidation(result)
        self._RefreshTaskTable()
        self._UpdateStateLabel()
        self.statusChanged.emit()

    def _RefreshValidation(self, result):
        self.ValidationTable.setRowCount(0)
        self.DerivedText.clear()
        self.LoadButton.setEnabled(False)
        if result is None:
            self.ValidationTable.setRowCount(1)
            self.ValidationTable.setItem(
                0, 0, QtWidgets.QTableWidgetItem("INFO")
            )
            self.ValidationTable.setItem(
                0, 1, QtWidgets.QTableWidgetItem("Draft")
            )
            self.ValidationTable.setItem(
                0, 2, QtWidgets.QTableWidgetItem(
                    "Press Validate Mission. Draft edits never alter the loaded snapshot."
                )
            )
            return

        self.ValidationTable.setRowCount(len(result.Issues))
        for row, issue in enumerate(result.Issues):
            colour = {
                "ERROR": "#ff6666",
                "WARNING": "#ffcc44",
                "INFO": "#bfbfbf",
            }.get(issue.Severity, "#ffffff")
            for column, value in enumerate(
                (issue.Severity, issue.Field, issue.Message)
            ):
                item = QtWidgets.QTableWidgetItem(str(value))
                if column == 0:
                    item.setForeground(QtGui.QColor(colour))
                self.ValidationTable.setItem(row, column, item)

        lines = []
        for task in self.DraftProfile.PrimaryTasks:
            derived = result.TaskDerived.get(task.TaskId, {})
            if not derived:
                continue
            lines.append(f"{task.Name} [{task.TaskId}]")
            lines.append(
                f"  PRI {derived.get('PriUs', 0.0):.1f} us | "
                f"CPI {derived.get('CpiMs', 0.0):.2f} ms | "
                f"CPI travel {derived.get('CpiTravelDeg', 0.0):.2f} deg | "
                f"dwell spacing {derived.get('DwellStartTravelDeg', 0.0):.2f} deg"
            )
            if isinstance(task, Scan360Task):
                lines.append(
                    f"  Revolution {derived.get('RevolutionSec', 0.0):.2f} s"
                )
            else:
                lines.append(
                    f"  Sector {derived.get('SectorWidthDeg', 0.0):.2f} deg | "
                    f"traverse {derived.get('TraverseSec', 0.0):.2f} s | "
                    f"revisit {derived.get('ExpectedRevisitSec', 0.0):.2f} s"
                )
        if result.TrackUpdateDerived:
            track = result.TrackUpdateDerived
            lines.extend([
                "Track Update",
                (
                    f"  {int(track.get('SampleCount', 0))} angular samples | "
                    f"acquisition {track.get('AcquisitionSec', 0.0):.2f} s | "
                    f"worst-case interruption "
                    f"{track.get('EstimatedInterruptionSec', 0.0):.2f} s"
                ),
                (
                    f"  requested revisit "
                    f"{track.get('RequestedRevisitSec', 0.0):.2f} s | "
                    f"earliest effective "
                    f"{track.get('EffectiveRevisitSec', 0.0):.2f} s"
                ),
            ])
        self.DerivedText.setPlainText("\n".join(lines))
        runtime_state = str(
            self.RuntimeStatus.get("State", "STOPPED")
        ).upper()
        self.LoadButton.setEnabled(
            result.IsValid
            and runtime_state not in (
                "RUNNING_360", "RUNNING_SECTOR", "PAUSED",
            )
        )

    def _LoadValidatedMission(self):
        try:
            loaded = self.Controller.LoadValidated()
        except RuntimeError as error:
            QtWidgets.QMessageBox.warning(self, "Mission not loaded", str(error))
            return
        self._QueueMissionCommand("LOAD", loaded)
        self.StateLabel.setText(
            f"LOAD requested: {loaded.Name} r{loaded.Revision}"
        )
        self.StateLabel.setStyleSheet("font-weight: bold; color: #ffcc44;")
        self.statusChanged.emit()

    def _StartMission(self):
        if str(self.RuntimeStatus.get("State", "")).upper() != "LOADED":
            return
        self._QueueMissionCommand("START")

    def _PauseOrResumeMission(self):
        state = str(self.RuntimeStatus.get("State", "")).upper()
        if state in ("RUNNING_360", "RUNNING_SECTOR"):
            self._QueueMissionCommand("PAUSE")
        elif state == "PAUSED":
            self._QueueMissionCommand("RESUME")

    def _NewMission(self):
        profile = ReconcileMissionWaveforms(
            CreateDefaultMission(),
            self.Validator.Limits.AvailableWaveformIds,
        )
        self.Controller = MissionController(
            self.Validator,
            profile,
        )
        self.SelectedTaskIndex = 0
        self._LoadMissionWidgets()
        self._DraftChanged()

    def _SaveMission(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Vanguard X mission",
            f"{self.DraftProfile.Name.replace(' ', '_')}.vxmission.json",
            "Vanguard X mission (*.vxmission.json);;JSON (*.json)",
        )
        if not path:
            return
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(
                MissionProfileToDict(self.DraftProfile),
                stream,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            stream.write("\n")
        self.StateLabel.setText(f"Saved draft: {os.path.basename(path)}")
        self.StateLabel.setStyleSheet("font-weight: bold; color: #bfbfbf;")

    def _OpenMission(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load Vanguard X mission draft",
            "",
            "Vanguard X mission (*.vxmission.json *.json);;All files (*)",
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as stream:
                profile = MissionProfileFromDict(json.load(stream))
        except Exception as error:
            QtWidgets.QMessageBox.critical(
                self, "Mission load failed", str(error),
            )
            return
        profile = ReconcileMissionWaveforms(
            profile,
            self.Validator.Limits.AvailableWaveformIds,
        )
        self.Controller = MissionController(self.Validator, profile)
        self.SelectedTaskIndex = 0
        self._LoadMissionWidgets()
        self._DraftChanged()
        self.StateLabel.setText(f"Opened draft: {os.path.basename(path)}")

    def _UpdateStateLabel(self):
        state = str(self.RuntimeStatus.get("State", "STOPPED")).upper()
        message = str(self.RuntimeStatus.get("Message", "")).strip()
        if state in ("RUNNING_360", "RUNNING_SECTOR", "PAUSED"):
            task_number = self.RuntimeStatus.get("ActiveTaskNumber")
            task_count = int(self.RuntimeStatus.get("TaskCount", 0))
            task_name = str(
                self.RuntimeStatus.get("ActiveTaskName", "Mission task")
            )
            elapsed = float(
                self.RuntimeStatus.get("ActiveTaskElapsedSec", 0.0)
            )
            remaining = self.RuntimeStatus.get("ActiveTaskRemainingSec")
            timing = (
                f"{elapsed:.1f} s active"
                if remaining is None
                else f"{elapsed:.1f} s active | {float(remaining):.1f} s remaining"
            )
            self.StateLabel.setText(
                f"{state} | task {task_number}/{task_count}: "
                f"{task_name} | {timing}"
            )
            self.StateLabel.setStyleSheet(
                "font-weight: bold; color: "
                + ("#ffcc44;" if state == "PAUSED" else "#00ff66;")
            )
        elif state == "LOADED":
            self.StateLabel.setText(message or "Mission loaded")
            self.StateLabel.setStyleSheet(
                "font-weight: bold; color: #00ff66;"
            )
        elif state in ("FAULTED", "ABORTED"):
            self.StateLabel.setText(message or state)
            self.StateLabel.setStyleSheet(
                "font-weight: bold; color: #ff6666;"
            )
        elif state == "COMPLETED":
            self.StateLabel.setText(message or "Mission completed")
            self.StateLabel.setStyleSheet(
                "font-weight: bold; color: #00ff66;"
            )
        else:
            loaded = self.Controller.LoadedProfile
            if loaded is not None:
                self.StateLabel.setText(
                    f"LOAD requested: {loaded.Name} r{loaded.Revision}"
                )
            else:
                self.StateLabel.setText("DRAFT only | validate and load")
            self.StateLabel.setStyleSheet(
                "font-weight: bold; color: #ffcc44;"
            )
