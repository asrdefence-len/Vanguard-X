"""
===============================================================================
Vanguard X-Band Radar Data Logger - Named File
===============================================================================

Logs radar detections and selected range-Doppler products to an HDF5 file that
can be read by MATLAB or Octave.

Recommended MATLAB / Octave reads:

    det = h5read('VanguardLog_....h5', '/detections/data');
    rd  = h5read('VanguardLog_....h5', '/range_doppler/magnitude_db');
    r   = h5read('VanguardLog_....h5', '/axes/range_m');
    v   = h5read('VanguardLog_....h5', '/axes/velocity_mps');

Detection columns are stored in the HDF5 attribute:

    h5readatt(filename, '/detections', 'columns')

The logger is deliberately independent of the display. It can be switched on or
off from the Config dictionary in VanguardxMain.py.
===============================================================================
"""

from __future__ import annotations

import json
import os
import time
from datetime import datetime
from typing import Any, Dict, Iterable, List, Optional

import numpy as np

try:
    import h5py
except Exception as exc:  # pragma: no cover
    h5py = None
    _H5PY_IMPORT_ERROR = exc
else:
    _H5PY_IMPORT_ERROR = None


class DataLogger:
    """
    HDF5 logger for Vanguard dwell products.

    The logger stores:
        /detections/data
            Rows of detection metadata.

        /range_doppler/magnitude_db
            Selected range-Doppler maps. This is optional and can be logged every
            N dwells to avoid huge files.

        /dwell/data
            One row per dwell containing scan metadata and counts.

        /axes/range_m and /axes/velocity_mps
            Written once when first available.
    """

    DETECTION_COLUMNS = [
        "dwell_id",
        "unix_time_s",
        "boresight_deg",
        "scan_cycle",
        "range_m",
        "velocity_mps",
        "amplitude_db",
        "snr_db",
        "range_bin",
        "doppler_bin",
    ]

    DWELL_COLUMNS = [
        "dwell_id",
        "unix_time_s",
        "boresight_deg",
        "scan_cycle",
        "num_detections",
        "peak_range_m",
        "peak_amplitude_db",
    ]

    def __init__(self, Config: Dict[str, Any]):
        if h5py is None:
            raise ImportError(
                "DataLogger requires h5py. Install with: python -m pip install h5py"
            ) from _H5PY_IMPORT_ERROR

        self.Config = Config
        self.Enabled = bool(Config.get("DataLoggingEnabled", False))

        self.LogDetections = bool(Config.get("LogDetections", True))
        self.LogRangeDoppler = bool(Config.get("LogRangeDoppler", True))
        self.LogGolayDiagnosticOnce = bool(
            Config.get("LogGolayDiagnosticOnce", False)
        )

        self.RangeDopplerEveryNDwells = int(Config.get("LogRangeDopplerEveryNDwells", 10))
        self.RangeDopplerMaxRangeM = float(Config.get("LogRangeDopplerMaxRangeM", Config.get("MaxDisplayRangeM", 15000.0)))
        self.RangeDopplerStoreFloat32 = bool(Config.get("LogRangeDopplerFloat32", True))

        self.LogDirectory = str(Config.get("DataLogDirectory", "DataLogs"))
        self.LogPrefix = str(Config.get("DataLogPrefix", "VanguardLog"))
        self.LogFilename = str(Config.get("DataLogFilename", "datafile1.h5"))
        self.DataLogOverwrite = bool(Config.get("DataLogOverwrite", True))
        self.FlushEveryNDwells = int(Config.get("DataLogFlushEveryNDwells", 25))

        self.File = None
        self.Filename = None
        self.DetectionDataset = None
        self.DwellDataset = None
        self.RdDataset = None
        self.RdDwellDataset = None
        self.RdBoresightDataset = None
        self.RdTimeDataset = None

        self.RangeAxisWritten = False
        self.VelocityAxisWritten = False
        self.RdShape = None
        self.DwellWriteCount = 0
        self.GolayDiagnosticWritten = False

        if self.Enabled:
            self._open_file()

    # ---------------------------------------------------------------------
    # Public API
    # ---------------------------------------------------------------------

    def update_control_state(self, ControlState: Optional[Dict[str, Any]]) -> None:
        """Apply operator logging controls from the display.

        Expected keys are:
            SaveDataEnabled: bool
            DataLogFilename: str

        If saving is switched on and no file is open, this opens the requested file.
        If saving is switched off, the file is flushed and closed.
        If the filename changes while saving is off, it is remembered for the next start.
        If the filename changes while saving is on, the current file is closed and a new
        one is opened using the new name.
        """
        if ControlState is None:
            return

        RequestedEnabled = bool(ControlState.get("SaveDataEnabled", self.Enabled))
        RequestedFilename = str(ControlState.get("DataLogFilename", self.LogFilename) or "datafile1.h5").strip()
        if RequestedFilename == "":
            RequestedFilename = "datafile1.h5"
        if not RequestedFilename.lower().endswith(".h5"):
            RequestedFilename += ".h5"

        FilenameChanged = RequestedFilename != self.LogFilename

        if FilenameChanged:
            self.LogFilename = RequestedFilename
            self.Config["DataLogFilename"] = self.LogFilename

        if RequestedEnabled and not self.Enabled:
            self.Enabled = True
            self.Config["DataLoggingEnabled"] = True
            self._open_file()
            return

        if (not RequestedEnabled) and self.Enabled:
            self.close()
            self.Enabled = False
            self.Config["DataLoggingEnabled"] = False
            return

        if RequestedEnabled and self.Enabled and FilenameChanged:
            # Start a new file with the requested name.
            self.close()
            self._open_file()

    def log_dwell(self, Processed: Any, Detections: Iterable[Any]) -> None:
        """Log products from one radar dwell."""

        if not self.Enabled or self.File is None:
            return

        Detections = list(Detections or [])
        NowS = time.time()

        DwellId = self._get_dwell_id(Processed)
        BoresightDeg = self._get_diag_float(Processed, "BoresightDeg", np.nan)
        ScanCycle = self._get_diag_float(Processed, "ScanCycle", np.nan)

        self._write_axes_if_needed(Processed)
        self._append_dwell_row(Processed, Detections, DwellId, NowS, BoresightDeg, ScanCycle)

        if self.LogDetections and len(Detections) > 0:
            self._append_detection_rows(Detections, DwellId, NowS, BoresightDeg, ScanCycle)

        if self.LogRangeDoppler:
            if self.RangeDopplerEveryNDwells > 0 and DwellId % self.RangeDopplerEveryNDwells == 0:
                self._append_range_doppler(Processed, DwellId, NowS, BoresightDeg)

        self._write_golay_diagnostic_once(Processed, DwellId, NowS)

        self.DwellWriteCount += 1
        if self.FlushEveryNDwells > 0 and self.DwellWriteCount % self.FlushEveryNDwells == 0:
            self.flush()

    def flush(self) -> None:
        if self.File is not None:
            self.File.flush()

    def close(self) -> None:
        if self.File is not None:
            self.File.flush()
            self.File.close()
            self.File = None

    # ---------------------------------------------------------------------
    # File setup
    # ---------------------------------------------------------------------

    def _open_file(self) -> None:
        os.makedirs(self.LogDirectory, exist_ok=True)

        # Reset per-file handles/state in case logging is restarted from the GUI.
        self.File = None
        self.Filename = None
        self.DetectionDataset = None
        self.DwellDataset = None
        self.RdDataset = None
        self.RdDwellDataset = None
        self.RdBoresightDataset = None
        self.RdTimeDataset = None
        self.RangeAxisWritten = False
        self.VelocityAxisWritten = False
        self.RdShape = None
        self.DwellWriteCount = 0
        self.GolayDiagnosticWritten = False

        RequestedFilename = str(getattr(self, "LogFilename", "") or "datafile1.h5").strip()
        if RequestedFilename:
            if not RequestedFilename.lower().endswith(".h5"):
                RequestedFilename += ".h5"
            if os.path.isabs(RequestedFilename):
                self.Filename = RequestedFilename
            else:
                self.Filename = os.path.join(self.LogDirectory, RequestedFilename)
        else:
            Timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.Filename = os.path.join(self.LogDirectory, f"{self.LogPrefix}_{Timestamp}.h5")

        if os.path.exists(self.Filename) and not self.DataLogOverwrite:
            Stem, Ext = os.path.splitext(self.Filename)
            Timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.Filename = f"{Stem}_{Timestamp}{Ext}"

        self.File = h5py.File(self.Filename, "w")
        self.File.attrs["created_local_time"] = datetime.now().isoformat(timespec="seconds")
        self.File.attrs["format"] = "Vanguard HDF5 radar log v1"
        self.File.attrs["config_json"] = self._safe_json_dumps(self.Config)

        DetGroup = self.File.create_group("detections")
        DetGroup.attrs["columns"] = ",".join(self.DETECTION_COLUMNS)
        self.DetectionDataset = DetGroup.create_dataset(
            "data",
            shape=(0, len(self.DETECTION_COLUMNS)),
            maxshape=(None, len(self.DETECTION_COLUMNS)),
            chunks=(1024, len(self.DETECTION_COLUMNS)),
            dtype="f8",
            compression="gzip",
            compression_opts=4,
        )

        DwellGroup = self.File.create_group("dwell")
        DwellGroup.attrs["columns"] = ",".join(self.DWELL_COLUMNS)
        self.DwellDataset = DwellGroup.create_dataset(
            "data",
            shape=(0, len(self.DWELL_COLUMNS)),
            maxshape=(None, len(self.DWELL_COLUMNS)),
            chunks=(1024, len(self.DWELL_COLUMNS)),
            dtype="f8",
            compression="gzip",
            compression_opts=4,
        )

        self.File.create_group("axes")
        RdGroup = self.File.create_group("range_doppler")
        RdGroup.attrs["description"] = "magnitude_db is stored as [profile_index, doppler_or_velocity_bin, range_bin]."

        self.RdDwellDataset = RdGroup.create_dataset(
            "dwell_id",
            shape=(0,),
            maxshape=(None,),
            chunks=(1024,),
            dtype="i8",
            compression="gzip",
            compression_opts=4,
        )
        self.RdBoresightDataset = RdGroup.create_dataset(
            "boresight_deg",
            shape=(0,),
            maxshape=(None,),
            chunks=(1024,),
            dtype="f8",
            compression="gzip",
            compression_opts=4,
        )
        self.RdTimeDataset = RdGroup.create_dataset(
            "unix_time_s",
            shape=(0,),
            maxshape=(None,),
            chunks=(1024,),
            dtype="f8",
            compression="gzip",
            compression_opts=4,
        )

        print(f"Data logging enabled: {self.Filename}")

    # ---------------------------------------------------------------------
    # Append helpers
    # ---------------------------------------------------------------------

    def _append_dwell_row(
        self,
        Processed: Any,
        Detections: List[Any],
        DwellId: int,
        NowS: float,
        BoresightDeg: float,
        ScanCycle: float,
    ) -> None:
        PeakRangeM = np.nan
        PeakAmpDb = np.nan

        if len(Detections) > 0:
            Amps = [self._get_detection_value(Det, ["AmplitudeDb", "PowerDb", "MagnitudeDb", "SnrDb"], np.nan) for Det in Detections]
            try:
                BestIndex = int(np.nanargmax(np.asarray(Amps, dtype=float)))
                PeakRangeM = self._get_detection_value(Detections[BestIndex], ["RangeM", "Range"], np.nan)
                PeakAmpDb = float(Amps[BestIndex])
            except Exception:
                pass

        Row = np.asarray([
            DwellId,
            NowS,
            BoresightDeg,
            ScanCycle,
            len(Detections),
            PeakRangeM,
            PeakAmpDb,
        ], dtype=float)

        self._append_rows(self.DwellDataset, Row.reshape(1, -1))

    def _append_detection_rows(
        self,
        Detections: List[Any],
        DwellId: int,
        NowS: float,
        BoresightDeg: float,
        ScanCycle: float,
    ) -> None:
        Rows = []
        for Det in Detections:
            RangeM = self._get_detection_value(Det, ["RangeM", "Range"], np.nan)
            VelocityMps = self._get_detection_value(Det, ["VelocityMps", "Velocity", "DopplerVelocityMps"], np.nan)
            AmplitudeDb = self._get_detection_value(Det, ["AmplitudeDb", "PowerDb", "MagnitudeDb"], np.nan)
            SnrDb = self._get_detection_value(Det, ["SnrDb", "SNRDb", "SignalToNoiseDb"], np.nan)
            RangeBin = self._get_detection_value(Det, ["RangeBin", "RangeIndex", "RangeCell"], np.nan)
            DopplerBin = self._get_detection_value(Det, ["DopplerBin", "DopplerIndex", "VelocityBin"], np.nan)

            Rows.append([
                DwellId,
                NowS,
                BoresightDeg,
                ScanCycle,
                RangeM,
                VelocityMps,
                AmplitudeDb,
                SnrDb,
                RangeBin,
                DopplerBin,
            ])

        if len(Rows) > 0:
            self._append_rows(self.DetectionDataset, np.asarray(Rows, dtype=float))

    def _append_range_doppler(self, Processed: Any, DwellId: int, NowS: float, BoresightDeg: float) -> None:
        if not hasattr(Processed, "MagnitudeDb"):
            return

        MagnitudeDb = np.asarray(Processed.MagnitudeDb)
        if MagnitudeDb.ndim != 2:
            return

        MagnitudeDb = self._limit_range_axis(MagnitudeDb, Processed)

        if self.RangeDopplerStoreFloat32:
            MagnitudeDb = MagnitudeDb.astype(np.float32, copy=False)
        else:
            MagnitudeDb = MagnitudeDb.astype(np.float64, copy=False)

        if self.RdDataset is None:
            self._create_rd_dataset(MagnitudeDb.shape, MagnitudeDb.dtype)

        if MagnitudeDb.shape != self.RdShape:
            # Do not corrupt the file with variable-shaped RD maps. This should
            # not happen in normal operation. If it does, log a warning and skip.
            print(
                f"Warning: skipped RD log for dwell {DwellId}; shape {MagnitudeDb.shape} "
                f"does not match initial RD shape {self.RdShape}."
            )
            return

        Index = self.RdDataset.shape[0]
        self.RdDataset.resize((Index + 1, self.RdShape[0], self.RdShape[1]))
        self.RdDataset[Index, :, :] = MagnitudeDb

        self._append_vector_value(self.RdDwellDataset, int(DwellId))
        self._append_vector_value(self.RdBoresightDataset, float(BoresightDeg))
        self._append_vector_value(self.RdTimeDataset, float(NowS))

    def _create_rd_dataset(self, RdShape: tuple, Dtype: Any) -> None:
        self.RdShape = tuple(RdShape)
        RdGroup = self.File["range_doppler"]
        self.RdDataset = RdGroup.create_dataset(
            "magnitude_db",
            shape=(0, self.RdShape[0], self.RdShape[1]),
            maxshape=(None, self.RdShape[0], self.RdShape[1]),
            chunks=(1, self.RdShape[0], self.RdShape[1]),
            dtype=Dtype,
            compression="gzip",
            compression_opts=4,
            shuffle=True,
        )

    def _write_golay_diagnostic_once(
        self,
        Processed: Any,
        DwellId: int,
        NowS: float,
    ) -> None:
        """Write the first complete fixed-point Golay dwell in this file."""

        if not self.LogGolayDiagnosticOnce or self.GolayDiagnosticWritten:
            return
        Payload = getattr(Processed, "GolayDiagnostic", None)
        if not isinstance(Payload, dict):
            return

        Diagnostics = getattr(Processed, "Diagnostics", {}) or {}
        if str(Diagnostics.get("ProcessingMode", "")).upper() != "GOLAY_COMPLEMENTARY":
            return
        if not bool(Diagnostics.get("RfTargetEmulatorActive", False)):
            return
        if bool(Diagnostics.get("RfTargetUseScenario", True)):
            return
        if abs(float(Diagnostics.get("RfTargetRadialVelocityMps", np.inf))) > 1.0e-9:
            return

        RequiredArrays = (
            "RawA",
            "RawB",
            "CompressedA",
            "CompressedB",
            "PulseValid",
        )
        if any(Name not in Payload for Name in RequiredArrays):
            return

        RawA = np.asarray(Payload["RawA"], dtype=np.complex64)
        RawB = np.asarray(Payload["RawB"], dtype=np.complex64)
        CompressedA = np.asarray(
            Payload["CompressedA"],
            dtype=np.complex64,
        )
        CompressedB = np.asarray(
            Payload["CompressedB"],
            dtype=np.complex64,
        )
        PulseValid = np.asarray(Payload["PulseValid"], dtype=bool).reshape(-1)
        if (
            RawA.ndim != 2
            or RawA.shape != RawB.shape
            or CompressedA.shape != CompressedB.shape
            or CompressedA.shape != RawA.shape
        ):
            return

        PhysicalPulseCount = 2 * RawA.shape[0]
        if PulseValid.size != PhysicalPulseCount or not np.all(PulseValid):
            return
        if int(Diagnostics.get("TransmitCommandCount", -1)) != PhysicalPulseCount:
            return
        if (
            int(Diagnostics.get("TransmitBurstAcknowledgementCount", -1))
            != PhysicalPulseCount
        ):
            return

        Group = self.File.create_group("golay_diagnostic")
        Group.attrs["description"] = (
            "One complete stationary fixed-point Golay dwell; arrays are "
            "[complementary_pair, range_sample]."
        )
        Group.attrs["dwell_id"] = int(DwellId)
        Group.attrs["unix_time_s"] = float(NowS)
        Group.attrs["waveform_a_id"] = str(Payload["WaveformAId"])
        Group.attrs["waveform_b_id"] = str(Payload["WaveformBId"])
        Group.attrs["sample_rate_hz"] = float(Payload["SampleRateHz"])
        Group.attrs["physical_pri_sec"] = float(Payload["PhysicalPriSec"])
        Group.attrs["pair_pri_sec"] = float(Payload["PairPriSec"])
        Group.attrs["samples_per_chip"] = int(Payload["SamplesPerChip"])
        Group.attrs["target_range_m"] = float(
            Diagnostics.get("RfTargetRangeM", np.nan)
        )
        Group.attrs["target_bearing_deg"] = float(
            Diagnostics.get("RfTargetBearingDeg", np.nan)
        )
        Group.attrs["target_radial_velocity_mps"] = float(
            Diagnostics.get("RfTargetRadialVelocityMps", np.nan)
        )
        Group.attrs["tx_gain_db"] = float(Diagnostics.get("TxGainDb", np.nan))
        Group.attrs["rx_gain_db"] = float(Diagnostics.get("RxGainDb", np.nan))
        Group.attrs["external_attenuation_db"] = float(
            Diagnostics.get("ExternalAttenuationDb", np.nan)
        )
        Group.attrs["transmit_command_count"] = int(
            Diagnostics["TransmitCommandCount"]
        )
        Group.attrs["transmit_acknowledgement_count"] = int(
            Diagnostics["TransmitBurstAcknowledgementCount"]
        )

        DatasetOptions = {
            "compression": "gzip",
            "compression_opts": 4,
            "shuffle": True,
        }
        Group.create_dataset("raw_a", data=RawA, **DatasetOptions)
        Group.create_dataset("raw_b", data=RawB, **DatasetOptions)
        Group.create_dataset(
            "compressed_a",
            data=CompressedA,
            **DatasetOptions,
        )
        Group.create_dataset(
            "compressed_b",
            data=CompressedB,
            **DatasetOptions,
        )
        Group.create_dataset("pulse_valid", data=PulseValid.astype(np.uint8))
        Group.create_dataset(
            "range_m",
            data=np.asarray(Processed.RangeAxisM, dtype=np.float64),
            compression="gzip",
            compression_opts=4,
        )

        self.GolayDiagnosticWritten = True
        self.File.flush()
        print(
            "Golay diagnostic captured once: "
            f"{self.Filename} dwell={DwellId}"
        )

    # ---------------------------------------------------------------------
    # Axes and range limiting
    # ---------------------------------------------------------------------

    def _write_axes_if_needed(self, Processed: Any) -> None:
        AxesGroup = self.File["axes"]

        if not self.RangeAxisWritten and hasattr(Processed, "RangeAxisM"):
            RangeAxisM = np.asarray(Processed.RangeAxisM, dtype=np.float64)
            RangeAxisM = self._limited_range_axis_vector(RangeAxisM)
            AxesGroup.create_dataset("range_m", data=RangeAxisM, compression="gzip", compression_opts=4)
            self.RangeAxisWritten = True

        if not self.VelocityAxisWritten and hasattr(Processed, "VelocityAxisMps"):
            VelocityAxisMps = np.asarray(Processed.VelocityAxisMps, dtype=np.float64)
            AxesGroup.create_dataset("velocity_mps", data=VelocityAxisMps, compression="gzip", compression_opts=4)
            self.VelocityAxisWritten = True

    def _limit_range_axis(self, MagnitudeDb: np.ndarray, Processed: Any) -> np.ndarray:
        if not hasattr(Processed, "RangeAxisM"):
            return MagnitudeDb

        RangeAxisM = np.asarray(Processed.RangeAxisM)
        if RangeAxisM.ndim != 1:
            return MagnitudeDb

        Mask = RangeAxisM <= self.RangeDopplerMaxRangeM
        if not np.any(Mask):
            return MagnitudeDb

        # Current processor/display convention is [doppler, range]. If the other
        # dimension matches range, handle that too.
        if MagnitudeDb.shape[1] == RangeAxisM.size:
            return MagnitudeDb[:, Mask]
        if MagnitudeDb.shape[0] == RangeAxisM.size:
            return MagnitudeDb[Mask, :]
        return MagnitudeDb

    def _limited_range_axis_vector(self, RangeAxisM: np.ndarray) -> np.ndarray:
        Mask = RangeAxisM <= self.RangeDopplerMaxRangeM
        if np.any(Mask):
            return RangeAxisM[Mask]
        return RangeAxisM

    # ---------------------------------------------------------------------
    # Generic low-level append helpers
    # ---------------------------------------------------------------------

    @staticmethod
    def _append_rows(Dataset: Any, Rows: np.ndarray) -> None:
        OldRows = Dataset.shape[0]
        NewRows = OldRows + Rows.shape[0]
        Dataset.resize((NewRows, Dataset.shape[1]))
        Dataset[OldRows:NewRows, :] = Rows

    @staticmethod
    def _append_vector_value(Dataset: Any, Value: Any) -> None:
        OldRows = Dataset.shape[0]
        Dataset.resize((OldRows + 1,))
        Dataset[OldRows] = Value

    # ---------------------------------------------------------------------
    # Attribute helpers
    # ---------------------------------------------------------------------

    @staticmethod
    def _get_dwell_id(Processed: Any) -> int:
        for Name in ["DwellId", "DwellID", "dwell_id"]:
            if hasattr(Processed, Name):
                try:
                    return int(getattr(Processed, Name))
                except Exception:
                    pass
        Diagnostics = getattr(Processed, "Diagnostics", {}) or {}
        for Name in ["DwellId", "DwellID", "dwell_id"]:
            if Name in Diagnostics:
                try:
                    return int(Diagnostics[Name])
                except Exception:
                    pass
        return -1

    @staticmethod
    def _get_diag_float(Processed: Any, Name: str, Default: float) -> float:
        Diagnostics = getattr(Processed, "Diagnostics", {}) or {}
        try:
            return float(Diagnostics.get(Name, Default))
        except Exception:
            return Default

    @staticmethod
    def _get_detection_value(Detection: Any, Names: List[str], Default: float) -> float:
        for Name in Names:
            if hasattr(Detection, Name):
                try:
                    return float(getattr(Detection, Name))
                except Exception:
                    pass
            if isinstance(Detection, dict) and Name in Detection:
                try:
                    return float(Detection[Name])
                except Exception:
                    pass
        return Default

    @staticmethod
    def _safe_json_dumps(Config: Dict[str, Any]) -> str:
        def Convert(Value: Any):
            if isinstance(Value, (str, int, float, bool)) or Value is None:
                return Value
            if isinstance(Value, np.generic):
                return Value.item()
            if isinstance(Value, (list, tuple)):
                return [Convert(Item) for Item in Value]
            if isinstance(Value, dict):
                return {str(Key): Convert(Val) for Key, Val in Value.items()}
            return str(Value)

        return json.dumps(Convert(Config), indent=2, sort_keys=True)


class NullDataLogger:
    """Drop-in logger used when logging is disabled."""

    Filename = None

    def update_control_state(self, ControlState: Optional[Dict[str, Any]]) -> None:
        """Apply operator logging controls from the display.

        Expected keys are:
            SaveDataEnabled: bool
            DataLogFilename: str

        If saving is switched on and no file is open, this opens the requested file.
        If saving is switched off, the file is flushed and closed.
        If the filename changes while saving is off, it is remembered for the next start.
        If the filename changes while saving is on, the current file is closed and a new
        one is opened using the new name.
        """
        if ControlState is None:
            return

        RequestedEnabled = bool(ControlState.get("SaveDataEnabled", self.Enabled))
        RequestedFilename = str(ControlState.get("DataLogFilename", self.LogFilename) or "datafile1.h5").strip()
        if RequestedFilename == "":
            RequestedFilename = "datafile1.h5"
        if not RequestedFilename.lower().endswith(".h5"):
            RequestedFilename += ".h5"

        FilenameChanged = RequestedFilename != self.LogFilename

        if FilenameChanged:
            self.LogFilename = RequestedFilename
            self.Config["DataLogFilename"] = self.LogFilename

        if RequestedEnabled and not self.Enabled:
            self.Enabled = True
            self.Config["DataLoggingEnabled"] = True
            self._open_file()
            return

        if (not RequestedEnabled) and self.Enabled:
            self.close()
            self.Enabled = False
            self.Config["DataLoggingEnabled"] = False
            return

        if RequestedEnabled and self.Enabled and FilenameChanged:
            # Start a new file with the requested name.
            self.close()
            self._open_file()

    def log_dwell(self, Processed: Any, Detections: Iterable[Any]) -> None:
        return

    def flush(self) -> None:
        return

    def close(self) -> None:
        return


def CreateDataLogger(Config: Dict[str, Any]):
    """Factory used by VanguardxMain.py."""

    if bool(Config.get("DataLoggingEnabled", False)):
        return DataLogger(Config)
    return NullDataLogger()
