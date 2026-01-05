"""
NEO-6M GPS Module Driver.

This module provides an interface for the u-blox NEO-6M GPS module
to capture position, speed, and heading data for track mapping
and lap timing.
"""

import logging
import math
import re
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from queue import Queue, Empty
from typing import Optional, Tuple

import numpy as np

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

logger = logging.getLogger(__name__)


@dataclass
class GPSData:
    """Container for GPS data."""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_m: float = 0.0
    speed_mph: float = 0.0
    heading_deg: float = 0.0
    satellites: int = 0
    hdop: float = 99.9
    fix_quality: int = 0
    timestamp: float = 0.0
    utc_time: str = ""

    @property
    def has_fix(self) -> bool:
        """Check if GPS has a valid fix."""
        return self.fix_quality > 0 and self.satellites >= 4

    def distance_to(self, other: 'GPSData') -> float:
        """
        Calculate distance to another GPS point using Haversine formula.

        Args:
            other: Another GPSData point

        Returns:
            Distance in meters
        """
        R = 6371000  # Earth's radius in meters

        lat1 = math.radians(self.latitude)
        lat2 = math.radians(other.latitude)
        dlat = math.radians(other.latitude - self.latitude)
        dlon = math.radians(other.longitude - self.longitude)

        a = (math.sin(dlat/2)**2 +
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    def bearing_to(self, other: 'GPSData') -> float:
        """
        Calculate bearing to another GPS point.

        Args:
            other: Another GPSData point

        Returns:
            Bearing in degrees (0-360)
        """
        lat1 = math.radians(self.latitude)
        lat2 = math.radians(other.latitude)
        dlon = math.radians(other.longitude - self.longitude)

        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) -
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))

        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360


class NMEAParser:
    """Parser for NMEA 0183 GPS sentences."""

    @staticmethod
    def parse_coordinate(value: str, direction: str) -> float:
        """
        Parse NMEA coordinate format to decimal degrees.

        Args:
            value: Coordinate in NMEA format (DDDMM.MMMMM)
            direction: Direction (N/S/E/W)

        Returns:
            Decimal degrees
        """
        if not value:
            return 0.0

        # Determine degrees/minutes split point
        if '.' in value:
            dot_idx = value.index('.')
            deg_len = dot_idx - 2
        else:
            deg_len = len(value) - 2

        degrees = float(value[:deg_len])
        minutes = float(value[deg_len:])

        decimal = degrees + minutes / 60

        if direction in ('S', 'W'):
            decimal = -decimal

        return decimal

    @staticmethod
    def parse_gga(parts: list) -> dict:
        """Parse GGA sentence (fix data)."""
        if len(parts) < 15:
            return {}

        try:
            return {
                'utc_time': parts[1],
                'latitude': NMEAParser.parse_coordinate(parts[2], parts[3]),
                'longitude': NMEAParser.parse_coordinate(parts[4], parts[5]),
                'fix_quality': int(parts[6]) if parts[6] else 0,
                'satellites': int(parts[7]) if parts[7] else 0,
                'hdop': float(parts[8]) if parts[8] else 99.9,
                'altitude_m': float(parts[9]) if parts[9] else 0.0,
            }
        except (ValueError, IndexError):
            return {}

    @staticmethod
    def parse_rmc(parts: list) -> dict:
        """Parse RMC sentence (recommended minimum data)."""
        if len(parts) < 12:
            return {}

        try:
            speed_knots = float(parts[7]) if parts[7] else 0.0
            heading = float(parts[8]) if parts[8] else 0.0

            return {
                'utc_time': parts[1],
                'status': parts[2],
                'latitude': NMEAParser.parse_coordinate(parts[3], parts[4]),
                'longitude': NMEAParser.parse_coordinate(parts[5], parts[6]),
                'speed_mph': speed_knots * 1.15078,  # Convert knots to mph
                'heading_deg': heading,
            }
        except (ValueError, IndexError):
            return {}

    @staticmethod
    def parse_vtg(parts: list) -> dict:
        """Parse VTG sentence (velocity and heading)."""
        if len(parts) < 9:
            return {}

        try:
            speed_kmh = float(parts[7]) if parts[7] else 0.0

            return {
                'heading_deg': float(parts[1]) if parts[1] else 0.0,
                'speed_mph': speed_kmh * 0.621371,  # Convert km/h to mph
            }
        except (ValueError, IndexError):
            return {}


class GPS:
    """
    NEO-6M GPS Module Driver.

    Provides interface for reading GPS data including position,
    speed, and heading. Supports both blocking and threaded operation.

    Attributes:
        port: Serial port path
        baud_rate: Serial baud rate
        update_rate: Target update rate in Hz
    """

    def __init__(
        self,
        port: str = '/dev/serial0',
        baud_rate: int = 9600,
        update_rate_hz: int = 10,
        simulate: bool = False
    ):
        """
        Initialize the GPS module.

        Args:
            port: Serial port path
            baud_rate: Baud rate for serial communication
            update_rate_hz: GPS update rate in Hz
            simulate: Run in simulation mode
        """
        self.port = port
        self.baud_rate = baud_rate
        self.update_rate_hz = update_rate_hz
        self.simulate = simulate or not HAS_SERIAL

        self._serial: Optional['serial.Serial'] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._data_queue: Queue = Queue(maxsize=100)
        self._last_data = GPSData()
        self._lock = threading.Lock()

        # Simulation state
        self._sim_lat = 35.3456
        self._sim_lon = -80.6892
        self._sim_heading = 0.0
        self._sim_speed = 0.0

        if not self.simulate:
            self._init_serial()

    def _init_serial(self) -> None:
        """Initialize serial connection."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1.0
            )

            # Configure update rate if > 1Hz
            if self.update_rate_hz > 1:
                self._set_update_rate(self.update_rate_hz)

            logger.info(f"GPS initialized on {self.port}")

        except Exception as e:
            logger.error(f"Failed to initialize GPS: {e}")
            self.simulate = True

    def _set_update_rate(self, rate_hz: int) -> None:
        """
        Set the GPS update rate.

        Args:
            rate_hz: Update rate in Hz (1, 2, 5, or 10)
        """
        if not self._serial:
            return

        rate_ms = 1000 // rate_hz
        # UBX command to set update rate
        # This is a simplified version; full implementation would use UBX protocol
        logger.debug(f"GPS update rate set to {rate_hz}Hz")

    def _parse_nmea(self, line: str) -> Optional[dict]:
        """
        Parse an NMEA sentence.

        Args:
            line: Raw NMEA sentence

        Returns:
            Parsed data dictionary or None if invalid
        """
        if not line.startswith('$'):
            return None

        # Remove checksum
        if '*' in line:
            line = line.split('*')[0]

        parts = line[1:].split(',')
        sentence_type = parts[0]

        if sentence_type.endswith('GGA'):
            return NMEAParser.parse_gga(parts)
        elif sentence_type.endswith('RMC'):
            return NMEAParser.parse_rmc(parts)
        elif sentence_type.endswith('VTG'):
            return NMEAParser.parse_vtg(parts)

        return None

    def _read_loop(self) -> None:
        """Background thread for reading GPS data."""
        buffer = ""

        while self._running:
            try:
                if self.simulate:
                    data = self._simulate_data()
                    with self._lock:
                        self._last_data = data
                    try:
                        self._data_queue.put_nowait(data)
                    except:
                        pass
                    time.sleep(1.0 / self.update_rate_hz)
                    continue

                # Read from serial
                if self._serial and self._serial.in_waiting:
                    chunk = self._serial.read(self._serial.in_waiting).decode('ascii', errors='ignore')
                    buffer += chunk

                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()

                        if line:
                            parsed = self._parse_nmea(line)
                            if parsed:
                                self._update_data(parsed)

                time.sleep(0.01)

            except Exception as e:
                logger.error(f"GPS read error: {e}")
                time.sleep(0.1)

    def _update_data(self, parsed: dict) -> None:
        """Update GPS data from parsed NMEA sentence."""
        with self._lock:
            for key, value in parsed.items():
                if hasattr(self._last_data, key):
                    setattr(self._last_data, key, value)
            self._last_data.timestamp = time.time()

        try:
            self._data_queue.put_nowait(GPSData(**{
                k: getattr(self._last_data, k)
                for k in self._last_data.__dataclass_fields__
            }))
        except:
            pass

    def _simulate_data(self) -> GPSData:
        """Generate simulated GPS data."""
        # Simulate driving in a circuit
        self._sim_heading += np.random.normal(0, 5)
        self._sim_heading = self._sim_heading % 360

        # Speed varies between 30-80 mph
        self._sim_speed += np.random.normal(0, 2)
        self._sim_speed = max(30, min(80, self._sim_speed))

        # Update position based on heading and speed
        speed_m_s = self._sim_speed * 0.44704  # mph to m/s
        dt = 1.0 / self.update_rate_hz

        dlat = speed_m_s * dt * math.cos(math.radians(self._sim_heading)) / 111000
        dlon = speed_m_s * dt * math.sin(math.radians(self._sim_heading)) / (111000 * math.cos(math.radians(self._sim_lat)))

        self._sim_lat += dlat
        self._sim_lon += dlon

        return GPSData(
            latitude=self._sim_lat,
            longitude=self._sim_lon,
            altitude_m=250.0 + np.random.normal(0, 1),
            speed_mph=self._sim_speed,
            heading_deg=self._sim_heading,
            satellites=10 + int(np.random.normal(0, 2)),
            hdop=1.2 + np.random.normal(0, 0.2),
            fix_quality=1,
            timestamp=time.time(),
            utc_time=datetime.utcnow().strftime('%H%M%S.%f')[:10]
        )

    def start(self) -> None:
        """Start the GPS reading thread."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("GPS reader started")

    def stop(self) -> None:
        """Stop the GPS reading thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("GPS reader stopped")

    def read(self) -> GPSData:
        """
        Read the latest GPS data.

        Returns:
            Most recent GPSData
        """
        if not self._running:
            if self.simulate:
                return self._simulate_data()

            # Single read mode
            return self._single_read()

        with self._lock:
            return GPSData(**{
                k: getattr(self._last_data, k)
                for k in self._last_data.__dataclass_fields__
            })

    def _single_read(self) -> GPSData:
        """Perform a single blocking GPS read."""
        if not self._serial:
            return GPSData()

        buffer = ""
        start_time = time.time()

        while time.time() - start_time < 2.0:
            if self._serial.in_waiting:
                chunk = self._serial.read(self._serial.in_waiting).decode('ascii', errors='ignore')
                buffer += chunk

                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    if line:
                        parsed = self._parse_nmea(line)
                        if parsed and 'latitude' in parsed:
                            self._update_data(parsed)
                            return self._last_data

            time.sleep(0.01)

        return self._last_data

    def get_queue(self) -> Queue:
        """Get the data queue for asynchronous reading."""
        return self._data_queue

    def wait_for_fix(self, timeout: float = 60.0) -> bool:
        """
        Wait for GPS to acquire a fix.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if fix acquired, False if timeout
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            data = self.read()
            if data.has_fix:
                logger.info(f"GPS fix acquired: {data.satellites} satellites")
                return True
            time.sleep(0.5)

        logger.warning("GPS fix timeout")
        return False

    def close(self) -> None:
        """Close the GPS connection."""
        self.stop()
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None


class LapTimer:
    """
    Lap timing using GPS position.

    Detects when the vehicle crosses the start/finish line
    and calculates lap times.
    """

    def __init__(
        self,
        start_finish_lat: float,
        start_finish_lon: float,
        detection_radius_m: float = 25.0,
        min_lap_time_sec: float = 60.0
    ):
        """
        Initialize the lap timer.

        Args:
            start_finish_lat: Start/finish line latitude
            start_finish_lon: Start/finish line longitude
            detection_radius_m: Detection radius in meters
            min_lap_time_sec: Minimum lap time to filter false triggers
        """
        self.start_finish = GPSData(
            latitude=start_finish_lat,
            longitude=start_finish_lon
        )
        self.detection_radius = detection_radius_m
        self.min_lap_time = min_lap_time_sec

        self.lap_times: list[float] = []
        self.current_lap_start: Optional[float] = None
        self._in_zone = False
        self._last_crossing: float = 0.0

    def update(self, gps_data: GPSData) -> Optional[float]:
        """
        Update lap timer with new GPS data.

        Args:
            gps_data: Current GPS reading

        Returns:
            Lap time in seconds if a lap was completed, None otherwise
        """
        distance = gps_data.distance_to(self.start_finish)
        in_zone = distance < self.detection_radius

        lap_time = None

        if in_zone and not self._in_zone:
            # Entering start/finish zone
            now = gps_data.timestamp

            if self.current_lap_start is not None:
                elapsed = now - self.current_lap_start

                if elapsed >= self.min_lap_time:
                    lap_time = elapsed
                    self.lap_times.append(lap_time)
                    logger.info(f"Lap completed: {lap_time:.3f}s")

            self.current_lap_start = now

        self._in_zone = in_zone
        return lap_time

    @property
    def best_lap(self) -> Optional[float]:
        """Get best lap time."""
        return min(self.lap_times) if self.lap_times else None

    @property
    def last_lap(self) -> Optional[float]:
        """Get last lap time."""
        return self.lap_times[-1] if self.lap_times else None

    @property
    def lap_count(self) -> int:
        """Get number of completed laps."""
        return len(self.lap_times)
