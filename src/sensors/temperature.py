"""
DS18B20 Temperature Sensor Driver.

This module provides an interface for reading DS18B20 one-wire
temperature sensors used to monitor shock absorber body temperatures
and ambient temperature.
"""

import glob
import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

logger = logging.getLogger(__name__)

# 1-Wire paths
W1_BASE_PATH = '/sys/bus/w1/devices'
W1_SLAVE_FILE = 'w1_slave'


@dataclass
class TemperatureReading:
    """Container for temperature reading."""
    temperature_f: float
    temperature_c: float
    sensor_id: str
    location: str
    timestamp: float
    valid: bool = True


class TemperatureSensor:
    """
    DS18B20 1-Wire Temperature Sensor Driver.

    Reads temperature from a DS18B20 sensor connected via
    the Raspberry Pi's 1-Wire interface.

    Attributes:
        sensor_id: Unique sensor ID (e.g., '28-0316a2795cff')
        location: Description of sensor location
    """

    def __init__(
        self,
        sensor_id: str,
        location: str = "",
        simulate: bool = False
    ):
        """
        Initialize the temperature sensor.

        Args:
            sensor_id: DS18B20 sensor ID
            location: Description of sensor mounting location
            simulate: Run in simulation mode
        """
        self.sensor_id = sensor_id
        self.location = location
        self.simulate = simulate

        self._device_path = os.path.join(W1_BASE_PATH, sensor_id, W1_SLAVE_FILE)
        self._last_temp: float = 75.0  # Initial sim value

        if not simulate and not os.path.exists(self._device_path):
            logger.warning(f"Temperature sensor not found: {sensor_id}")
            self.simulate = True

    def read_raw(self) -> Optional[str]:
        """
        Read raw data from sensor.

        Returns:
            Raw sensor output or None if failed
        """
        if self.simulate:
            return None

        try:
            with open(self._device_path, 'r') as f:
                return f.read()
        except Exception as e:
            logger.debug(f"Temperature read error: {e}")
            return None

    def read(self) -> TemperatureReading:
        """
        Read temperature from sensor.

        Returns:
            TemperatureReading with temperature in F and C
        """
        timestamp = time.time()

        if self.simulate:
            return self._simulate_reading(timestamp)

        raw_data = self.read_raw()

        if raw_data is None:
            return TemperatureReading(
                temperature_f=0.0,
                temperature_c=0.0,
                sensor_id=self.sensor_id,
                location=self.location,
                timestamp=timestamp,
                valid=False
            )

        # Check CRC
        lines = raw_data.strip().split('\n')
        if len(lines) < 2 or 'YES' not in lines[0]:
            return TemperatureReading(
                temperature_f=0.0,
                temperature_c=0.0,
                sensor_id=self.sensor_id,
                location=self.location,
                timestamp=timestamp,
                valid=False
            )

        # Parse temperature
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos + 2:]
            try:
                temp_c = float(temp_string) / 1000.0
                temp_f = temp_c * 9.0 / 5.0 + 32.0

                return TemperatureReading(
                    temperature_f=temp_f,
                    temperature_c=temp_c,
                    sensor_id=self.sensor_id,
                    location=self.location,
                    timestamp=timestamp,
                    valid=True
                )
            except ValueError:
                pass

        return TemperatureReading(
            temperature_f=0.0,
            temperature_c=0.0,
            sensor_id=self.sensor_id,
            location=self.location,
            timestamp=timestamp,
            valid=False
        )

    def _simulate_reading(self, timestamp: float) -> TemperatureReading:
        """Generate simulated temperature reading."""
        # Simulate gradual temperature change
        self._last_temp += np.random.normal(0, 0.5)

        # Different baseline temps for different locations
        if 'shock' in self.location.lower():
            target = 140.0  # Shocks run warm
        elif 'ambient' in self.location.lower():
            target = 75.0
        else:
            target = 100.0

        # Drift toward target
        self._last_temp += (target - self._last_temp) * 0.01

        temp_f = self._last_temp
        temp_c = (temp_f - 32) * 5 / 9

        return TemperatureReading(
            temperature_f=temp_f,
            temperature_c=temp_c,
            sensor_id=self.sensor_id,
            location=self.location,
            timestamp=timestamp,
            valid=True
        )


class TemperatureArray:
    """
    Array of DS18B20 temperature sensors.

    Manages multiple temperature sensors for monitoring
    shock absorber temperatures and ambient conditions.
    """

    def __init__(self, config: dict, simulate: bool = False):
        """
        Initialize temperature sensor array from configuration.

        Args:
            config: Configuration dictionary from sensor_config.json
            simulate: Run in simulation mode
        """
        self.simulate = simulate
        self.sensors: Dict[str, TemperatureSensor] = {}

        sensor_config = config.get('sensors', {})

        for name, cfg in sensor_config.items():
            self.sensors[name] = TemperatureSensor(
                sensor_id=cfg['id'],
                location=cfg.get('location', name),
                simulate=simulate
            )

        logger.info(f"Initialized {len(self.sensors)} temperature sensors")

    def read_all(self) -> Dict[str, TemperatureReading]:
        """
        Read all temperature sensors.

        Returns:
            Dictionary mapping sensor names to readings
        """
        readings = {}
        for name, sensor in self.sensors.items():
            readings[name] = sensor.read()
        return readings

    def read_temperatures_f(self) -> Dict[str, float]:
        """
        Read all temperatures in Fahrenheit.

        Returns:
            Dictionary mapping sensor names to temperatures
        """
        readings = self.read_all()
        return {
            name: r.temperature_f
            for name, r in readings.items()
            if r.valid
        }

    def read_temperatures_c(self) -> Dict[str, float]:
        """
        Read all temperatures in Celsius.

        Returns:
            Dictionary mapping sensor names to temperatures
        """
        readings = self.read_all()
        return {
            name: r.temperature_c
            for name, r in readings.items()
            if r.valid
        }

    def get_shock_temps(self) -> Dict[str, float]:
        """
        Get shock absorber temperatures.

        Returns:
            Dictionary mapping corner names to temperatures in F
        """
        temps = {}
        for name, sensor in self.sensors.items():
            if 'shock' in name.lower():
                reading = sensor.read()
                if reading.valid:
                    corner = name.replace('shock_', '')
                    temps[corner] = reading.temperature_f
        return temps

    def get_ambient_temp(self) -> Optional[float]:
        """
        Get ambient temperature.

        Returns:
            Ambient temperature in F or None if not available
        """
        if 'ambient' in self.sensors:
            reading = self.sensors['ambient'].read()
            if reading.valid:
                return reading.temperature_f
        return None

    @staticmethod
    def discover_sensors() -> List[str]:
        """
        Discover connected DS18B20 sensors.

        Returns:
            List of sensor IDs
        """
        sensors = []

        if os.path.exists(W1_BASE_PATH):
            for device in glob.glob(os.path.join(W1_BASE_PATH, '28-*')):
                sensor_id = os.path.basename(device)
                sensors.append(sensor_id)
                logger.info(f"Discovered temperature sensor: {sensor_id}")

        return sensors

    def close(self) -> None:
        """Close all sensor connections."""
        # DS18B20 doesn't require explicit close
        pass


class TemperatureMonitor:
    """
    Threaded temperature monitoring with averaging.

    Provides smoothed temperature readings with configurable
    update rate and averaging window.
    """

    def __init__(
        self,
        sensors: TemperatureArray,
        update_rate_hz: float = 1.0,
        averaging_window: int = 5
    ):
        """
        Initialize temperature monitor.

        Args:
            sensors: TemperatureArray instance
            update_rate_hz: Update rate in Hz
            averaging_window: Number of samples to average
        """
        self.sensors = sensors
        self.update_rate = update_rate_hz
        self.averaging_window = averaging_window

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._history: Dict[str, List[float]] = {}
        self._averaged: Dict[str, float] = {}

    def _update_loop(self) -> None:
        """Background thread for reading temperatures."""
        while self._running:
            try:
                readings = self.sensors.read_temperatures_f()

                with self._lock:
                    for name, temp in readings.items():
                        if name not in self._history:
                            self._history[name] = []

                        self._history[name].append(temp)

                        # Keep only recent samples
                        if len(self._history[name]) > self.averaging_window:
                            self._history[name] = self._history[name][-self.averaging_window:]

                        # Calculate average
                        self._averaged[name] = np.mean(self._history[name])

                time.sleep(1.0 / self.update_rate)

            except Exception as e:
                logger.error(f"Temperature monitor error: {e}")
                time.sleep(1.0)

    def start(self) -> None:
        """Start the temperature monitor."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()
        logger.info("Temperature monitor started")

    def stop(self) -> None:
        """Stop the temperature monitor."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("Temperature monitor stopped")

    def get_temperatures(self) -> Dict[str, float]:
        """
        Get averaged temperatures.

        Returns:
            Dictionary mapping sensor names to averaged temperatures
        """
        with self._lock:
            return dict(self._averaged)

    def get_temperature(self, name: str) -> Optional[float]:
        """
        Get averaged temperature for a specific sensor.

        Args:
            name: Sensor name

        Returns:
            Averaged temperature or None if not available
        """
        with self._lock:
            return self._averaged.get(name)
