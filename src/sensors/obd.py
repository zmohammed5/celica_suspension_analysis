"""
OBD-II Interface via ELM327.

This module provides an interface for reading vehicle data
through the OBD-II diagnostic port using an ELM327 adapter.
Supports standard PIDs and custom manufacturer-specific PIDs.
"""

import logging
import threading
import time
from dataclasses import dataclass
from queue import Queue
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import obd
    HAS_OBD = True
except ImportError:
    HAS_OBD = False

logger = logging.getLogger(__name__)


@dataclass
class OBDData:
    """Container for OBD-II data."""
    speed_mph: float = 0.0
    rpm: float = 0.0
    throttle_pct: float = 0.0
    coolant_temp_f: float = 0.0
    intake_temp_f: float = 0.0
    maf_gps: float = 0.0
    fuel_level_pct: float = 0.0
    steering_angle_deg: float = 0.0
    brake_pressure_psi: float = 0.0
    timestamp: float = 0.0

    @property
    def speed_kph(self) -> float:
        """Speed in km/h."""
        return self.speed_mph * 1.60934


class OBDInterface:
    """
    ELM327 OBD-II Interface.

    Provides access to vehicle ECU data through the OBD-II
    diagnostic port. Supports both Bluetooth and USB adapters.

    Attributes:
        port: Serial port or Bluetooth address
        protocol: OBD protocol (auto-detect by default)
        supported_pids: List of supported OBD PIDs
    """

    # Standard OBD-II PIDs
    PIDS = {
        'SPEED': ('01', '0D'),
        'RPM': ('01', '0C'),
        'THROTTLE_POS': ('01', '11'),
        'COOLANT_TEMP': ('01', '05'),
        'INTAKE_TEMP': ('01', '0F'),
        'MAF': ('01', '10'),
        'FUEL_LEVEL': ('01', '2F'),
    }

    def __init__(
        self,
        port: str = '/dev/rfcomm0',
        protocol: Optional[str] = None,
        fast_mode: bool = True,
        simulate: bool = False
    ):
        """
        Initialize the OBD interface.

        Args:
            port: Serial port or Bluetooth MAC address
            protocol: OBD protocol (None for auto-detect)
            fast_mode: Use fast mode for faster polling
            simulate: Run in simulation mode
        """
        self.port = port
        self.protocol = protocol
        self.fast_mode = fast_mode
        self.simulate = simulate or not HAS_OBD

        self._connection: Optional['obd.OBD'] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._data_queue: Queue = Queue(maxsize=100)
        self._last_data = OBDData()
        self._lock = threading.Lock()

        self.supported_pids: List[str] = []
        self._custom_pids: Dict[str, dict] = {}

        # Simulation state
        self._sim_rpm = 2500
        self._sim_throttle = 25.0

        if not self.simulate:
            self._connect()

    def _connect(self) -> bool:
        """
        Establish connection to ELM327 adapter.

        Returns:
            True if connection successful
        """
        try:
            self._connection = obd.OBD(
                portstr=self.port,
                fast=self.fast_mode,
                timeout=10
            )

            if self._connection.is_connected():
                logger.info(f"OBD connected: {self._connection.port_name()}")
                self._detect_supported_pids()
                return True
            else:
                logger.warning("OBD connection failed")
                self.simulate = True
                return False

        except Exception as e:
            logger.error(f"OBD connection error: {e}")
            self.simulate = True
            return False

    def _detect_supported_pids(self) -> None:
        """Detect which PIDs are supported by the vehicle."""
        if not self._connection:
            return

        for name, (mode, pid) in self.PIDS.items():
            try:
                cmd = obd.commands[name] if hasattr(obd.commands, name) else None
                if cmd and self._connection.supports(cmd):
                    self.supported_pids.append(name)
            except Exception:
                pass

        logger.info(f"Supported PIDs: {self.supported_pids}")

    def add_custom_pid(
        self,
        name: str,
        mode: str,
        pid: str,
        formula: str,
        unit: str = ""
    ) -> None:
        """
        Add a custom/manufacturer-specific PID.

        Args:
            name: PID name
            mode: OBD mode (e.g., '21' for manufacturer-specific)
            pid: PID code
            formula: Formula to decode response (uses A, B, C, D for bytes)
            unit: Unit of measurement
        """
        self._custom_pids[name] = {
            'mode': mode,
            'pid': pid,
            'formula': formula,
            'unit': unit
        }
        logger.debug(f"Added custom PID: {name}")

    def _query_pid(self, name: str) -> Optional[float]:
        """
        Query a standard OBD PID.

        Args:
            name: PID name

        Returns:
            Decoded value or None if failed
        """
        if not self._connection or name not in self.supported_pids:
            return None

        try:
            cmd = getattr(obd.commands, name, None)
            if cmd:
                response = self._connection.query(cmd)
                if response.value is not None:
                    return response.value.magnitude
        except Exception as e:
            logger.debug(f"PID query failed: {name} - {e}")

        return None

    def _query_custom_pid(self, name: str) -> Optional[float]:
        """
        Query a custom PID.

        Args:
            name: Custom PID name

        Returns:
            Decoded value or None if failed
        """
        if name not in self._custom_pids:
            return None

        pid_config = self._custom_pids[name]

        try:
            # Build command
            cmd_str = pid_config['mode'] + pid_config['pid']

            # Send raw command
            response = self._connection.query(
                obd.commands.CUSTOM, cmd_str
            )

            if response.value:
                # Parse response bytes
                data = response.value
                if len(data) >= 2:
                    A = data[0] if len(data) > 0 else 0
                    B = data[1] if len(data) > 1 else 0
                    C = data[2] if len(data) > 2 else 0
                    D = data[3] if len(data) > 3 else 0

                    # Evaluate formula
                    value = eval(pid_config['formula'])
                    return float(value)

        except Exception as e:
            logger.debug(f"Custom PID query failed: {name} - {e}")

        return None

    def _read_all(self) -> OBDData:
        """Read all available OBD data."""
        timestamp = time.time()

        if self.simulate:
            return self._simulate_data(timestamp)

        data = OBDData(timestamp=timestamp)

        # Query standard PIDs
        speed = self._query_pid('SPEED')
        if speed is not None:
            data.speed_mph = speed * 0.621371  # km/h to mph

        rpm = self._query_pid('RPM')
        if rpm is not None:
            data.rpm = rpm

        throttle = self._query_pid('THROTTLE_POS')
        if throttle is not None:
            data.throttle_pct = throttle

        coolant = self._query_pid('COOLANT_TEMP')
        if coolant is not None:
            data.coolant_temp_f = coolant * 9/5 + 32  # C to F

        intake = self._query_pid('INTAKE_TEMP')
        if intake is not None:
            data.intake_temp_f = intake * 9/5 + 32

        maf = self._query_pid('MAF')
        if maf is not None:
            data.maf_gps = maf

        fuel = self._query_pid('FUEL_LEVEL')
        if fuel is not None:
            data.fuel_level_pct = fuel

        # Query custom PIDs
        steering = self._query_custom_pid('steering_angle')
        if steering is not None:
            data.steering_angle_deg = steering

        brake = self._query_custom_pid('brake_pressure')
        if brake is not None:
            data.brake_pressure_psi = brake

        return data

    def _simulate_data(self, timestamp: float) -> OBDData:
        """Generate simulated OBD data."""
        # Simulate realistic driving
        self._sim_throttle += np.random.normal(0, 5)
        self._sim_throttle = max(0, min(100, self._sim_throttle))

        self._sim_rpm += np.random.normal(0, 200)
        self._sim_rpm = max(800, min(7000, self._sim_rpm))

        # Correlate speed with RPM/throttle
        target_speed = 40 + (self._sim_throttle * 0.5) + (self._sim_rpm / 100)
        current_speed = target_speed + np.random.normal(0, 2)

        return OBDData(
            speed_mph=max(0, current_speed),
            rpm=self._sim_rpm,
            throttle_pct=self._sim_throttle,
            coolant_temp_f=195 + np.random.normal(0, 2),
            intake_temp_f=85 + np.random.normal(0, 3),
            maf_gps=15 + self._sim_throttle * 0.3,
            fuel_level_pct=75 + np.random.normal(0, 0.5),
            steering_angle_deg=np.random.normal(0, 30),
            brake_pressure_psi=max(0, np.random.normal(0, 20)),
            timestamp=timestamp
        )

    def _read_loop(self) -> None:
        """Background thread for reading OBD data."""
        while self._running:
            try:
                data = self._read_all()

                with self._lock:
                    self._last_data = data

                try:
                    self._data_queue.put_nowait(data)
                except:
                    pass

                # OBD is slow, ~10Hz max
                time.sleep(0.1)

            except Exception as e:
                logger.error(f"OBD read error: {e}")
                time.sleep(1.0)

    def start(self) -> None:
        """Start the OBD reading thread."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("OBD reader started")

    def stop(self) -> None:
        """Stop the OBD reading thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("OBD reader stopped")

    def read(self) -> OBDData:
        """
        Read the latest OBD data.

        Returns:
            Most recent OBDData
        """
        if self._running:
            with self._lock:
                return OBDData(
                    speed_mph=self._last_data.speed_mph,
                    rpm=self._last_data.rpm,
                    throttle_pct=self._last_data.throttle_pct,
                    coolant_temp_f=self._last_data.coolant_temp_f,
                    intake_temp_f=self._last_data.intake_temp_f,
                    maf_gps=self._last_data.maf_gps,
                    fuel_level_pct=self._last_data.fuel_level_pct,
                    steering_angle_deg=self._last_data.steering_angle_deg,
                    brake_pressure_psi=self._last_data.brake_pressure_psi,
                    timestamp=self._last_data.timestamp
                )
        else:
            return self._read_all()

    def get_queue(self) -> Queue:
        """Get the data queue for asynchronous reading."""
        return self._data_queue

    def read_dtc(self) -> List[Tuple[str, str]]:
        """
        Read diagnostic trouble codes.

        Returns:
            List of (code, description) tuples
        """
        if self.simulate or not self._connection:
            return []

        try:
            response = self._connection.query(obd.commands.GET_DTC)
            if response.value:
                return [(code, desc) for code, desc in response.value]
        except Exception as e:
            logger.error(f"DTC read error: {e}")

        return []

    def clear_dtc(self) -> bool:
        """
        Clear diagnostic trouble codes.

        Returns:
            True if successful
        """
        if self.simulate or not self._connection:
            return False

        try:
            response = self._connection.query(obd.commands.CLEAR_DTC)
            return response.is_null() is False
        except Exception as e:
            logger.error(f"DTC clear error: {e}")
            return False

    def get_vin(self) -> Optional[str]:
        """
        Read Vehicle Identification Number.

        Returns:
            VIN string or None
        """
        if self.simulate:
            return "JTDDR32T4Y0012345"

        if not self._connection:
            return None

        try:
            response = self._connection.query(obd.commands.VIN)
            if response.value:
                return str(response.value)
        except Exception:
            pass

        return None

    @property
    def is_connected(self) -> bool:
        """Check if OBD is connected."""
        if self.simulate:
            return True
        return self._connection is not None and self._connection.is_connected()

    def close(self) -> None:
        """Close the OBD connection."""
        self.stop()
        if self._connection:
            try:
                self._connection.close()
            except Exception:
                pass
            self._connection = None
