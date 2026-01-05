"""
Data Export and Session Management.

This module provides functionality for managing data sessions,
exporting data to various formats, and handling data storage.
"""

import csv
import gzip
import json
import logging
import os
import shutil
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class SessionMetadata:
    """Metadata for a data recording session."""
    session_id: str
    start_time: str
    end_time: Optional[str] = None
    duration_sec: float = 0.0
    sample_count: int = 0
    sample_rate_hz: float = 100.0
    vehicle: str = ""
    driver: str = ""
    track: str = ""
    conditions: str = ""
    setup_notes: str = ""
    tags: List[str] = None

    def __post_init__(self):
        if self.tags is None:
            self.tags = []


class DataExporter:
    """
    Export data to various file formats.

    Supports CSV, JSON, and compressed formats for
    efficient storage of large datasets.
    """

    def __init__(self, output_dir: str = 'data/sessions'):
        """
        Initialize data exporter.

        Args:
            output_dir: Base directory for data storage
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def export_csv(
        self,
        data: List[Dict[str, Any]],
        filepath: str,
        compress: bool = False
    ) -> str:
        """
        Export data to CSV format.

        Args:
            data: List of data dictionaries
            filepath: Output file path
            compress: Whether to gzip compress the output

        Returns:
            Path to created file
        """
        if not data:
            logger.warning("No data to export")
            return ""

        filepath = Path(filepath)

        # Get all unique columns
        columns = []
        for row in data:
            for key in row.keys():
                if key not in columns:
                    columns.append(key)

        if compress:
            filepath = filepath.with_suffix('.csv.gz')
            open_func = lambda p: gzip.open(p, 'wt', newline='')
        else:
            filepath = filepath.with_suffix('.csv')
            open_func = lambda p: open(p, 'w', newline='')

        with open_func(filepath) as f:
            writer = csv.DictWriter(f, fieldnames=columns)
            writer.writeheader()
            writer.writerows(data)

        logger.info(f"Exported {len(data)} rows to {filepath}")
        return str(filepath)

    def export_json(
        self,
        data: Union[Dict, List],
        filepath: str,
        compress: bool = False,
        indent: int = 2
    ) -> str:
        """
        Export data to JSON format.

        Args:
            data: Data to export
            filepath: Output file path
            compress: Whether to gzip compress the output
            indent: JSON indentation (None for compact)

        Returns:
            Path to created file
        """
        filepath = Path(filepath)

        if compress:
            filepath = filepath.with_suffix('.json.gz')
            with gzip.open(filepath, 'wt') as f:
                json.dump(data, f, indent=indent, default=str)
        else:
            filepath = filepath.with_suffix('.json')
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=indent, default=str)

        logger.info(f"Exported to {filepath}")
        return str(filepath)

    def export_numpy(
        self,
        data: Dict[str, np.ndarray],
        filepath: str,
        compress: bool = True
    ) -> str:
        """
        Export data as numpy archive.

        Args:
            data: Dictionary of numpy arrays
            filepath: Output file path
            compress: Whether to compress

        Returns:
            Path to created file
        """
        filepath = Path(filepath)

        if compress:
            filepath = filepath.with_suffix('.npz')
            np.savez_compressed(filepath, **data)
        else:
            filepath = filepath.with_suffix('.npy')
            np.save(filepath, data)

        logger.info(f"Exported to {filepath}")
        return str(filepath)

    def load_csv(
        self,
        filepath: str
    ) -> List[Dict[str, Any]]:
        """
        Load data from CSV file.

        Args:
            filepath: Path to CSV file

        Returns:
            List of data dictionaries
        """
        filepath = Path(filepath)

        if filepath.suffix == '.gz':
            open_func = lambda p: gzip.open(p, 'rt')
        else:
            open_func = lambda p: open(p, 'r')

        with open_func(filepath) as f:
            reader = csv.DictReader(f)
            data = list(reader)

        # Convert numeric strings to floats
        for row in data:
            for key, value in row.items():
                try:
                    row[key] = float(value)
                except (ValueError, TypeError):
                    pass

        logger.info(f"Loaded {len(data)} rows from {filepath}")
        return data


class SessionManager:
    """
    Manage data recording sessions.

    Handles session creation, data buffering, and automatic
    file writing for continuous data logging.
    """

    # CSV column order
    COLUMNS = [
        'timestamp', 'elapsed_ms',
        'pot_fl_mm', 'pot_fr_mm', 'pot_rl_mm', 'pot_rr_mm',
        'accel_fl_z_g', 'accel_fr_z_g', 'accel_rl_z_g', 'accel_rr_z_g',
        'body_accel_x_g', 'body_accel_y_g', 'body_accel_z_g',
        'body_roll_deg', 'body_pitch_deg', 'body_yaw_rate_dps',
        'gps_lat', 'gps_lon', 'gps_speed_mph', 'gps_heading',
        'steering_angle_deg', 'throttle_pct', 'brake_pressure_psi',
        'temp_shock_fl_f', 'temp_shock_fr_f', 'temp_shock_rl_f', 'temp_shock_rr_f',
        'temp_ambient_f',
        'calculated_wheel_speed_fl', 'calculated_wheel_speed_fr',
        'calculated_wheel_speed_rl', 'calculated_wheel_speed_rr',
    ]

    def __init__(
        self,
        data_dir: str = 'data/sessions',
        buffer_size: int = 1000,
        write_interval_sec: float = 1.0
    ):
        """
        Initialize session manager.

        Args:
            data_dir: Base directory for session data
            buffer_size: Number of samples to buffer before writing
            write_interval_sec: Maximum time between writes
        """
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)

        self.buffer_size = buffer_size
        self.write_interval = write_interval_sec

        self._current_session: Optional[str] = None
        self._metadata: Optional[SessionMetadata] = None
        self._buffer: List[Dict[str, Any]] = []
        self._file_handle = None
        self._csv_writer = None
        self._lock = threading.Lock()
        self._write_thread: Optional[threading.Thread] = None
        self._running = False
        self._start_time: float = 0.0
        self._sample_count: int = 0
        self._markers: List[Dict[str, Any]] = []

    @property
    def session_dir(self) -> Optional[Path]:
        """Get current session directory."""
        if self._current_session:
            return self.data_dir / self._current_session
        return None

    def start_session(
        self,
        vehicle: str = "Toyota Celica GT-S",
        driver: str = "",
        track: str = "",
        conditions: str = "",
        setup_notes: str = ""
    ) -> str:
        """
        Start a new data recording session.

        Args:
            vehicle: Vehicle description
            driver: Driver name
            track: Track/location name
            conditions: Weather/track conditions
            setup_notes: Setup configuration notes

        Returns:
            Session ID
        """
        if self._running:
            self.stop_session()

        # Generate session ID
        session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        session_dir = self.data_dir / session_id
        session_dir.mkdir(parents=True, exist_ok=True)

        # Create metadata
        self._metadata = SessionMetadata(
            session_id=session_id,
            start_time=datetime.now().isoformat(),
            vehicle=vehicle,
            driver=driver,
            track=track,
            conditions=conditions,
            setup_notes=setup_notes
        )

        # Open data file
        data_file = session_dir / 'data.csv'
        self._file_handle = open(data_file, 'w', newline='')
        self._csv_writer = csv.DictWriter(
            self._file_handle,
            fieldnames=self.COLUMNS,
            extrasaction='ignore'
        )
        self._csv_writer.writeheader()

        self._current_session = session_id
        self._buffer = []
        self._start_time = time.time()
        self._sample_count = 0
        self._markers = []
        self._running = True

        # Start background write thread
        self._write_thread = threading.Thread(target=self._write_loop, daemon=True)
        self._write_thread.start()

        logger.info(f"Started session: {session_id}")
        return session_id

    def add_sample(self, data: Dict[str, Any]) -> None:
        """
        Add a data sample to the current session.

        Args:
            data: Sample data dictionary
        """
        if not self._running:
            return

        # Add timestamp info
        data['timestamp'] = time.time()
        data['elapsed_ms'] = (time.time() - self._start_time) * 1000

        with self._lock:
            self._buffer.append(data)
            self._sample_count += 1

    def add_marker(self, label: str, notes: str = "") -> None:
        """
        Add a marker/event to the session.

        Args:
            label: Marker label
            notes: Additional notes
        """
        marker = {
            'timestamp': time.time(),
            'elapsed_ms': (time.time() - self._start_time) * 1000,
            'label': label,
            'notes': notes
        }

        with self._lock:
            self._markers.append(marker)

        logger.info(f"Marker added: {label}")

    def _write_loop(self) -> None:
        """Background thread for writing data to disk."""
        last_write = time.time()

        while self._running:
            time.sleep(0.1)

            should_write = (
                len(self._buffer) >= self.buffer_size or
                time.time() - last_write >= self.write_interval
            )

            if should_write and self._buffer:
                self._flush_buffer()
                last_write = time.time()

        # Final flush
        if self._buffer:
            self._flush_buffer()

    def _flush_buffer(self) -> None:
        """Write buffered data to disk."""
        with self._lock:
            if not self._buffer:
                return

            data_to_write = self._buffer
            self._buffer = []

        try:
            for row in data_to_write:
                self._csv_writer.writerow(row)
            self._file_handle.flush()

            logger.debug(f"Flushed {len(data_to_write)} samples")

        except Exception as e:
            logger.error(f"Error writing data: {e}")

    def stop_session(self) -> Optional[SessionMetadata]:
        """
        Stop the current session.

        Returns:
            Session metadata
        """
        if not self._running:
            return None

        self._running = False

        # Wait for write thread
        if self._write_thread:
            self._write_thread.join(timeout=5.0)
            self._write_thread = None

        # Final flush
        self._flush_buffer()

        # Close file
        if self._file_handle:
            self._file_handle.close()
            self._file_handle = None
            self._csv_writer = None

        # Update metadata
        if self._metadata:
            self._metadata.end_time = datetime.now().isoformat()
            self._metadata.duration_sec = time.time() - self._start_time
            self._metadata.sample_count = self._sample_count

            # Save metadata
            self._save_metadata()

            # Save markers
            if self._markers:
                self._save_markers()

        logger.info(f"Stopped session: {self._current_session} ({self._sample_count} samples)")

        metadata = self._metadata
        self._current_session = None
        self._metadata = None

        return metadata

    def _save_metadata(self) -> None:
        """Save session metadata to JSON file."""
        if not self.session_dir or not self._metadata:
            return

        filepath = self.session_dir / 'metadata.json'
        with open(filepath, 'w') as f:
            json.dump(asdict(self._metadata), f, indent=2)

    def _save_markers(self) -> None:
        """Save session markers to JSON file."""
        if not self.session_dir or not self._markers:
            return

        filepath = self.session_dir / 'markers.json'
        with open(filepath, 'w') as f:
            json.dump(self._markers, f, indent=2)

    def list_sessions(self) -> List[Dict[str, Any]]:
        """
        List all available sessions.

        Returns:
            List of session info dictionaries
        """
        sessions = []

        for session_dir in sorted(self.data_dir.iterdir(), reverse=True):
            if not session_dir.is_dir():
                continue

            metadata_file = session_dir / 'metadata.json'
            if metadata_file.exists():
                with open(metadata_file, 'r') as f:
                    metadata = json.load(f)
                    sessions.append(metadata)
            else:
                # Basic info from directory
                sessions.append({
                    'session_id': session_dir.name,
                    'start_time': None,
                    'sample_count': 0
                })

        return sessions

    def load_session(self, session_id: str) -> Dict[str, Any]:
        """
        Load session data and metadata.

        Args:
            session_id: Session ID to load

        Returns:
            Dictionary with 'metadata', 'data', and 'markers'
        """
        session_dir = self.data_dir / session_id

        if not session_dir.exists():
            raise ValueError(f"Session not found: {session_id}")

        result = {
            'metadata': None,
            'data': [],
            'markers': []
        }

        # Load metadata
        metadata_file = session_dir / 'metadata.json'
        if metadata_file.exists():
            with open(metadata_file, 'r') as f:
                result['metadata'] = json.load(f)

        # Load data
        data_file = session_dir / 'data.csv'
        if data_file.exists():
            exporter = DataExporter()
            result['data'] = exporter.load_csv(str(data_file))

        # Load markers
        markers_file = session_dir / 'markers.json'
        if markers_file.exists():
            with open(markers_file, 'r') as f:
                result['markers'] = json.load(f)

        return result

    def delete_session(self, session_id: str) -> bool:
        """
        Delete a session.

        Args:
            session_id: Session ID to delete

        Returns:
            True if deleted successfully
        """
        session_dir = self.data_dir / session_id

        if session_dir.exists():
            shutil.rmtree(session_dir)
            logger.info(f"Deleted session: {session_id}")
            return True

        return False

    @property
    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._running

    @property
    def current_session_id(self) -> Optional[str]:
        """Get current session ID."""
        return self._current_session

    @property
    def elapsed_time(self) -> float:
        """Get elapsed recording time in seconds."""
        if self._running:
            return time.time() - self._start_time
        return 0.0

    @property
    def sample_count(self) -> int:
        """Get current sample count."""
        return self._sample_count
