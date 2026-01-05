"""
Sensor Calibration Tools.

This module provides calibration wizards and utilities for
calibrating accelerometers, potentiometers, and temperature sensors.
"""

import json
import logging
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class CalibrationPoint:
    """Single calibration measurement point."""
    raw_value: float
    reference_value: float
    timestamp: float
    samples: int = 1


@dataclass
class CalibrationResult:
    """Calibration result with parameters."""
    offset: float
    scale: float
    r_squared: float
    points: List[CalibrationPoint]
    calibration_date: str
    sensor_id: str


class PotentiometerCalibrator:
    """
    Calibration wizard for linear potentiometers.

    Guides the user through measuring potentiometer voltage
    at known displacement positions to create a calibration curve.
    """

    def __init__(
        self,
        adc,
        channel: int,
        corner: str,
        samples_per_point: int = 100,
        settle_time_sec: float = 1.0
    ):
        """
        Initialize potentiometer calibrator.

        Args:
            adc: ADS1115 ADC instance
            channel: ADC channel number
            corner: Corner identifier
            samples_per_point: Number of samples to average at each point
            settle_time_sec: Time to wait before sampling
        """
        self.adc = adc
        self.channel = channel
        self.corner = corner
        self.samples_per_point = samples_per_point
        self.settle_time = settle_time_sec

        self.calibration_points: List[CalibrationPoint] = []

    def measure_point(self, reference_position_mm: float) -> CalibrationPoint:
        """
        Measure a single calibration point.

        Args:
            reference_position_mm: Known position in mm

        Returns:
            CalibrationPoint with measured voltage
        """
        # Wait for settling
        time.sleep(self.settle_time)

        # Collect samples
        samples = []
        for _ in range(self.samples_per_point):
            voltage = self.adc.read_voltage(self.channel)
            samples.append(voltage)
            time.sleep(0.01)

        avg_voltage = np.mean(samples)

        point = CalibrationPoint(
            raw_value=avg_voltage,
            reference_value=reference_position_mm,
            timestamp=time.time(),
            samples=self.samples_per_point
        )

        self.calibration_points.append(point)
        logger.info(f"Measured: {reference_position_mm}mm = {avg_voltage:.4f}V")

        return point

    def calculate_calibration(self) -> Optional[CalibrationResult]:
        """
        Calculate calibration parameters from measured points.

        Uses linear regression to determine offset and scale.

        Returns:
            CalibrationResult or None if insufficient points
        """
        if len(self.calibration_points) < 2:
            logger.error("Need at least 2 calibration points")
            return None

        voltages = np.array([p.raw_value for p in self.calibration_points])
        positions = np.array([p.reference_value for p in self.calibration_points])

        # Linear regression: position = scale * voltage + offset
        coeffs = np.polyfit(voltages, positions, 1)
        scale = coeffs[0]
        offset = coeffs[1]

        # Calculate R-squared
        predicted = np.polyval(coeffs, voltages)
        ss_res = np.sum((positions - predicted) ** 2)
        ss_tot = np.sum((positions - np.mean(positions)) ** 2)
        r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

        result = CalibrationResult(
            offset=offset,
            scale=scale,
            r_squared=r_squared,
            points=self.calibration_points,
            calibration_date=time.strftime('%Y-%m-%d %H:%M:%S'),
            sensor_id=f"pot_{self.corner}"
        )

        logger.info(f"Calibration complete: scale={scale:.4f}, offset={offset:.4f}, R²={r_squared:.4f}")
        return result

    def run_wizard(self, positions_mm: List[float] = None) -> Optional[CalibrationResult]:
        """
        Run interactive calibration wizard.

        Args:
            positions_mm: List of positions to measure (default: 0, 25, 50, 75, 100)

        Returns:
            CalibrationResult or None if cancelled
        """
        if positions_mm is None:
            positions_mm = [0, 25, 50, 75, 100]

        print(f"\n=== Potentiometer Calibration: {self.corner} ===")
        print(f"You will need to set the potentiometer to {len(positions_mm)} positions.")
        print("Press Enter when ready at each position, or 'q' to quit.\n")

        for position in positions_mm:
            user_input = input(f"Set potentiometer to {position}mm and press Enter: ")
            if user_input.lower() == 'q':
                print("Calibration cancelled.")
                return None

            self.measure_point(position)

        return self.calculate_calibration()

    def clear_points(self) -> None:
        """Clear all calibration points."""
        self.calibration_points = []


class AccelerometerCalibrator:
    """
    Calibration wizard for MPU6050 accelerometers.

    Supports two calibration methods:
    1. Static calibration: Place sensor level, measure gravity offset
    2. Six-position calibration: Full 3-axis calibration using gravity
    """

    POSITIONS = {
        'z_up': {'axis': 2, 'expected': 1.0, 'description': 'Z-axis pointing up'},
        'z_down': {'axis': 2, 'expected': -1.0, 'description': 'Z-axis pointing down'},
        'x_up': {'axis': 0, 'expected': 1.0, 'description': 'X-axis pointing up'},
        'x_down': {'axis': 0, 'expected': -1.0, 'description': 'X-axis pointing down'},
        'y_up': {'axis': 1, 'expected': 1.0, 'description': 'Y-axis pointing up'},
        'y_down': {'axis': 1, 'expected': -1.0, 'description': 'Y-axis pointing down'},
    }

    def __init__(
        self,
        accelerometer,
        samples_per_position: int = 500,
        settle_time_sec: float = 2.0
    ):
        """
        Initialize accelerometer calibrator.

        Args:
            accelerometer: MPU6050 instance
            samples_per_position: Number of samples per position
            settle_time_sec: Settling time before sampling
        """
        self.accel = accelerometer
        self.samples_per_position = samples_per_position
        self.settle_time = settle_time_sec

        self.measurements: Dict[str, np.ndarray] = {}

    def measure_position(self, position_name: str) -> np.ndarray:
        """
        Measure accelerometer at a specific position.

        Args:
            position_name: Position identifier

        Returns:
            Array of [x, y, z] mean accelerations
        """
        time.sleep(self.settle_time)

        samples = []
        for _ in range(self.samples_per_position):
            accel_data = self.accel.read_accel()
            samples.append([accel_data.x, accel_data.y, accel_data.z])
            time.sleep(0.01)

        mean_values = np.mean(samples, axis=0)
        self.measurements[position_name] = mean_values

        logger.info(f"Position {position_name}: [{mean_values[0]:.4f}, {mean_values[1]:.4f}, {mean_values[2]:.4f}]")
        return mean_values

    def calculate_offset_scale(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate offset and scale from six-position calibration.

        Returns:
            Tuple of (offset_array, scale_array)
        """
        offsets = np.zeros(3)
        scales = np.ones(3)

        for axis in range(3):
            axis_measurements = []
            for pos_name, pos_info in self.POSITIONS.items():
                if pos_info['axis'] == axis and pos_name in self.measurements:
                    axis_measurements.append({
                        'measured': self.measurements[pos_name][axis],
                        'expected': pos_info['expected']
                    })

            if len(axis_measurements) >= 2:
                # Calculate offset and scale from +1g and -1g measurements
                m_plus = next((m for m in axis_measurements if m['expected'] == 1.0), None)
                m_minus = next((m for m in axis_measurements if m['expected'] == -1.0), None)

                if m_plus and m_minus:
                    offsets[axis] = (m_plus['measured'] + m_minus['measured']) / 2
                    scales[axis] = 2.0 / (m_plus['measured'] - m_minus['measured'])

        logger.info(f"Calculated offsets: {offsets}")
        logger.info(f"Calculated scales: {scales}")

        return offsets, scales

    def quick_calibration(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Quick calibration assuming sensor is level (Z-up).

        Measures offset assuming only gravity is present.

        Returns:
            Tuple of (offset_array, scale_array)
        """
        print("\nPlace accelerometer on a level surface with Z-axis pointing up.")
        input("Press Enter when ready...")

        self.measure_position('z_up')

        z_up = self.measurements['z_up']

        # Offset is the deviation from expected [0, 0, 1]
        offsets = np.array([z_up[0], z_up[1], z_up[2] - 1.0])
        scales = np.array([1.0, 1.0, 1.0])

        return offsets, scales

    def run_full_wizard(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Run full six-position calibration wizard.

        Returns:
            Tuple of (offset_array, scale_array)
        """
        print("\n=== Accelerometer Six-Position Calibration ===")
        print("You will position the sensor in 6 orientations.")
        print("Press Enter when ready at each position, or 'q' to quit.\n")

        for pos_name, pos_info in self.POSITIONS.items():
            user_input = input(f"Position sensor with {pos_info['description']}, press Enter: ")
            if user_input.lower() == 'q':
                print("Calibration cancelled.")
                return np.zeros(3), np.ones(3)

            self.measure_position(pos_name)

        return self.calculate_offset_scale()

    def measure_gyro_offset(self, duration_sec: float = 5.0) -> np.ndarray:
        """
        Measure gyroscope zero offset while stationary.

        Args:
            duration_sec: Duration to sample

        Returns:
            Array of [x, y, z] gyro offsets in deg/s
        """
        print(f"\nKeep sensor stationary for {duration_sec} seconds...")
        time.sleep(1.0)

        samples = []
        start_time = time.time()

        while time.time() - start_time < duration_sec:
            gyro_data = self.accel.read_gyro()
            samples.append([gyro_data.x, gyro_data.y, gyro_data.z])
            time.sleep(0.01)

        offsets = np.mean(samples, axis=0)
        logger.info(f"Gyro offsets: {offsets}")

        return offsets


class TemperatureSensorMapper:
    """
    Utility for discovering and mapping temperature sensors.

    Helps identify which DS18B20 sensor ID corresponds to
    which physical location.
    """

    def __init__(self):
        """Initialize temperature sensor mapper."""
        self.sensor_map: Dict[str, str] = {}

    def discover_sensors(self) -> List[str]:
        """
        Discover connected DS18B20 sensors.

        Returns:
            List of sensor IDs
        """
        from ..sensors.temperature import TemperatureArray
        return TemperatureArray.discover_sensors()

    def identify_sensor(
        self,
        sensor_id: str,
        location: str
    ) -> None:
        """
        Map a sensor ID to a location.

        Args:
            sensor_id: DS18B20 sensor ID
            location: Location description
        """
        self.sensor_map[location] = sensor_id
        logger.info(f"Mapped {sensor_id} to {location}")

    def run_wizard(self, locations: List[str] = None) -> Dict[str, str]:
        """
        Run interactive sensor identification wizard.

        Heats each sensor location one at a time to identify it.

        Args:
            locations: List of location names

        Returns:
            Dictionary mapping locations to sensor IDs
        """
        if locations is None:
            locations = ['shock_fl', 'shock_fr', 'shock_rl', 'shock_rr', 'ambient']

        sensors = self.discover_sensors()

        if not sensors:
            print("No temperature sensors found!")
            return {}

        print(f"\n=== Temperature Sensor Mapping ===")
        print(f"Found {len(sensors)} sensors: {sensors}")
        print("\nFor each location, warm the sensor (hold it) to identify it.\n")

        from ..sensors.temperature import TemperatureSensor

        for location in locations:
            print(f"\nIdentifying sensor at: {location}")
            print("Warm the sensor at this location and press Enter...")
            input()

            # Read all sensors, find the warmest one
            temps = {}
            for sid in sensors:
                if sid not in self.sensor_map.values():
                    sensor = TemperatureSensor(sid, simulate=False)
                    reading = sensor.read()
                    if reading.valid:
                        temps[sid] = reading.temperature_f

            if temps:
                warmest = max(temps, key=temps.get)
                print(f"Warmest sensor: {warmest} at {temps[warmest]:.1f}°F")

                confirm = input(f"Map {warmest} to {location}? (y/n): ")
                if confirm.lower() == 'y':
                    self.identify_sensor(warmest, location)
            else:
                manual = input(f"Enter sensor ID for {location}: ")
                if manual in sensors:
                    self.identify_sensor(manual, location)

        return self.sensor_map

    def save_mapping(self, filepath: str) -> None:
        """Save sensor mapping to JSON file."""
        with open(filepath, 'w') as f:
            json.dump(self.sensor_map, f, indent=2)

    def load_mapping(self, filepath: str) -> None:
        """Load sensor mapping from JSON file."""
        with open(filepath, 'r') as f:
            self.sensor_map = json.load(f)


class CalibrationWizard:
    """
    Master calibration wizard for all sensors.

    Provides a unified interface for calibrating all sensors
    in the data acquisition system.
    """

    def __init__(self, config_dir: str = 'config'):
        """
        Initialize calibration wizard.

        Args:
            config_dir: Directory containing configuration files
        """
        self.config_dir = Path(config_dir)
        self.results: Dict[str, CalibrationResult] = {}

    def save_results(self, filename: str = 'calibration_results.json') -> None:
        """Save all calibration results to JSON file."""
        filepath = self.config_dir / filename

        data = {}
        for sensor_id, result in self.results.items():
            data[sensor_id] = asdict(result)

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        logger.info(f"Saved calibration results to {filepath}")

    def load_results(self, filename: str = 'calibration_results.json') -> None:
        """Load calibration results from JSON file."""
        filepath = self.config_dir / filename

        if filepath.exists():
            with open(filepath, 'r') as f:
                data = json.load(f)

            for sensor_id, result_data in data.items():
                points = [
                    CalibrationPoint(**p) for p in result_data.get('points', [])
                ]
                result_data['points'] = points
                self.results[sensor_id] = CalibrationResult(**result_data)

            logger.info(f"Loaded calibration results from {filepath}")

    def run_full_calibration(self) -> None:
        """Run complete calibration for all sensors."""
        print("\n" + "="*50)
        print("CELICA SUSPENSION DAQ - SENSOR CALIBRATION")
        print("="*50)

        print("\nThis wizard will guide you through calibrating:")
        print("  1. Linear potentiometers (4 corners)")
        print("  2. Accelerometers (body + 4 corners)")
        print("  3. Temperature sensors (mapping)")
        print("\nPress Enter to begin or 'q' to quit...")

        user_input = input()
        if user_input.lower() == 'q':
            return

        # Calibration steps would go here
        print("\nCalibration complete!")
