"""
Linear Potentiometer Driver with ADS1115 ADC.

This module provides interfaces for reading linear potentiometers used
to measure shock/damper displacement at each corner of the vehicle.
The potentiometers are read through ADS1115 16-bit ADC modules.
"""

import logging
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import smbus2
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False

logger = logging.getLogger(__name__)


# ADS1115 Register Addresses
ADS1115_CONVERSION = 0x00
ADS1115_CONFIG = 0x01
ADS1115_LO_THRESH = 0x02
ADS1115_HI_THRESH = 0x03

# ADS1115 Configuration bits
ADS1115_OS_SINGLE = 0x8000
ADS1115_MUX_AIN0 = 0x4000
ADS1115_MUX_AIN1 = 0x5000
ADS1115_MUX_AIN2 = 0x6000
ADS1115_MUX_AIN3 = 0x7000

ADS1115_GAIN = {
    2/3: 0x0000,  # +/- 6.144V
    1: 0x0200,    # +/- 4.096V
    2: 0x0400,    # +/- 2.048V
    4: 0x0600,    # +/- 1.024V
    8: 0x0800,    # +/- 0.512V
    16: 0x0A00,   # +/- 0.256V
}

ADS1115_GAIN_VOLTAGE = {
    2/3: 6.144,
    1: 4.096,
    2: 2.048,
    4: 1.024,
    8: 0.512,
    16: 0.256,
}

ADS1115_DATA_RATE = {
    8: 0x0000,
    16: 0x0020,
    32: 0x0040,
    64: 0x0060,
    128: 0x0080,
    250: 0x00A0,
    475: 0x00C0,
    860: 0x00E0,
}


@dataclass
class PotentiometerReading:
    """Container for potentiometer reading."""
    position_mm: float
    velocity_mm_s: float
    voltage: float
    timestamp: float
    corner: str


class ADS1115:
    """
    ADS1115 16-bit ADC Driver.

    Provides high-resolution analog readings for linear potentiometers.
    """

    def __init__(
        self,
        address: int = 0x48,
        bus_num: int = 1,
        gain: float = 1,
        data_rate: int = 860,
        simulate: bool = False
    ):
        """
        Initialize the ADS1115 ADC.

        Args:
            address: I2C address (0x48-0x4B based on ADDR pin)
            bus_num: I2C bus number
            gain: PGA gain setting (2/3, 1, 2, 4, 8, or 16)
            data_rate: Samples per second (8, 16, 32, 64, 128, 250, 475, 860)
            simulate: Run in simulation mode
        """
        self.address = address
        self.bus_num = bus_num
        self.gain = gain
        self.data_rate = data_rate
        self.simulate = simulate or not HAS_SMBUS

        self._bus: Optional['smbus2.SMBus'] = None
        self._voltage_scale = ADS1115_GAIN_VOLTAGE[gain] / 32768.0

        if not self.simulate:
            self._init_device()

    def _init_device(self) -> None:
        """Initialize the ADC."""
        try:
            self._bus = smbus2.SMBus(self.bus_num)
            logger.info(f"ADS1115 initialized at 0x{self.address:02X}")
        except Exception as e:
            logger.error(f"Failed to initialize ADS1115: {e}")
            self.simulate = True

    def read_adc(self, channel: int = 0) -> int:
        """
        Read raw ADC value from specified channel.

        Args:
            channel: Input channel (0-3)

        Returns:
            Signed 16-bit ADC value
        """
        if self.simulate:
            # Simulate mid-range voltage with noise
            return int(16384 + np.random.normal(0, 100))

        mux = [ADS1115_MUX_AIN0, ADS1115_MUX_AIN1,
               ADS1115_MUX_AIN2, ADS1115_MUX_AIN3][channel]

        config = (
            ADS1115_OS_SINGLE |
            mux |
            ADS1115_GAIN[self.gain] |
            0x0100 |  # Single-shot mode
            ADS1115_DATA_RATE[self.data_rate] |
            0x0003    # Disable comparator
        )

        # Write config and start conversion
        self._bus.write_i2c_block_data(
            self.address,
            ADS1115_CONFIG,
            [(config >> 8) & 0xFF, config & 0xFF]
        )

        # Wait for conversion
        delay = 1.0 / self.data_rate + 0.001
        time.sleep(delay)

        # Read result
        data = self._bus.read_i2c_block_data(self.address, ADS1115_CONVERSION, 2)
        value = (data[0] << 8) | data[1]

        # Convert to signed
        if value >= 0x8000:
            value -= 0x10000

        return value

    def read_voltage(self, channel: int = 0) -> float:
        """
        Read voltage from specified channel.

        Args:
            channel: Input channel (0-3)

        Returns:
            Voltage in volts
        """
        raw = self.read_adc(channel)
        return raw * self._voltage_scale

    def close(self) -> None:
        """Close the I2C bus connection."""
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None


class Potentiometer:
    """
    Linear Potentiometer with ADS1115 ADC.

    Converts voltage readings to linear displacement in millimeters
    using calibration data.
    """

    def __init__(
        self,
        adc: ADS1115,
        channel: int,
        corner: str,
        calibration: dict,
        simulate: bool = False
    ):
        """
        Initialize the potentiometer.

        Args:
            adc: ADS1115 ADC instance
            channel: ADC channel (0-3)
            corner: Corner identifier (e.g., 'front_left')
            calibration: Calibration dictionary with voltage/position mappings
            simulate: Run in simulation mode
        """
        self.adc = adc
        self.channel = channel
        self.corner = corner
        self.simulate = simulate

        # Calibration parameters
        self.voltage_min = calibration.get('voltage_min', 0.25)
        self.voltage_max = calibration.get('voltage_max', 4.75)
        self.position_min = calibration.get('position_min_mm', 0)
        self.position_max = calibration.get('position_max_mm', 100)
        self.zero_offset = calibration.get('zero_offset_mm', 50)

        # Calculate scale factor
        voltage_range = self.voltage_max - self.voltage_min
        position_range = self.position_max - self.position_min
        self.scale = position_range / voltage_range

        # For velocity calculation
        self._last_position: Optional[float] = None
        self._last_time: Optional[float] = None

        # Simulation state
        self._sim_position = self.zero_offset
        self._sim_velocity = 0.0

    def read_raw_voltage(self) -> float:
        """Read raw voltage from ADC."""
        if self.simulate:
            return self._simulate_voltage()
        return self.adc.read_voltage(self.channel)

    def read_position(self) -> Tuple[float, float]:
        """
        Read current position in millimeters.

        Position is relative to the calibrated zero point,
        with positive values indicating compression and
        negative values indicating extension.

        Returns:
            Tuple of (position_mm, timestamp)
        """
        timestamp = time.time()
        voltage = self.read_raw_voltage()

        # Convert voltage to position
        position = (voltage - self.voltage_min) * self.scale + self.position_min

        # Apply zero offset (relative to ride height)
        position = position - self.zero_offset

        # Clamp to valid range
        position = max(
            self.position_min - self.zero_offset,
            min(self.position_max - self.zero_offset, position)
        )

        return position, timestamp

    def read(self) -> PotentiometerReading:
        """
        Read position and calculate velocity.

        Returns:
            PotentiometerReading with position, velocity, and metadata
        """
        position, timestamp = self.read_position()
        voltage = self.read_raw_voltage()

        # Calculate velocity
        velocity = 0.0
        if self._last_position is not None and self._last_time is not None:
            dt = timestamp - self._last_time
            if dt > 0:
                velocity = (position - self._last_position) / dt

        self._last_position = position
        self._last_time = timestamp

        return PotentiometerReading(
            position_mm=position,
            velocity_mm_s=velocity,
            voltage=voltage,
            timestamp=timestamp,
            corner=self.corner
        )

    def _simulate_voltage(self) -> float:
        """Generate simulated voltage readings."""
        # Random walk for position
        self._sim_velocity += np.random.normal(0, 50)
        self._sim_velocity *= 0.95  # Damping
        self._sim_position += self._sim_velocity * 0.01

        # Keep within bounds
        if self._sim_position < self.position_min:
            self._sim_position = self.position_min
            self._sim_velocity = abs(self._sim_velocity)
        elif self._sim_position > self.position_max:
            self._sim_position = self.position_max
            self._sim_velocity = -abs(self._sim_velocity)

        # Convert to voltage
        voltage = (
            self.voltage_min +
            (self._sim_position - self.position_min) / self.scale
        )

        # Add noise
        voltage += np.random.normal(0, 0.005)

        return voltage

    def reset_velocity(self) -> None:
        """Reset velocity calculation state."""
        self._last_position = None
        self._last_time = None


class PotentiometerArray:
    """
    Array of four potentiometers for all vehicle corners.

    Provides synchronized readings from all corners and
    calculates derived metrics like body motion.
    """

    CORNERS = ['front_left', 'front_right', 'rear_left', 'rear_right']

    def __init__(self, config: dict, simulate: bool = False):
        """
        Initialize potentiometer array from configuration.

        Args:
            config: Configuration dictionary from sensor_config.json
            simulate: Run in simulation mode
        """
        self.simulate = simulate
        self.potentiometers: Dict[str, Potentiometer] = {}
        self._adcs: Dict[str, ADS1115] = {}

        adc_config = config.get('adc_modules', {})

        for corner in self.CORNERS:
            if corner not in adc_config:
                logger.warning(f"No ADC config for {corner}")
                continue

            cfg = adc_config[corner]

            # Create ADC for this corner
            adc = ADS1115(
                address=int(cfg['address'], 16),
                gain=cfg.get('gain', 1),
                data_rate=cfg.get('sample_rate', 860),
                simulate=simulate
            )
            self._adcs[corner] = adc

            # Create potentiometer
            self.potentiometers[corner] = Potentiometer(
                adc=adc,
                channel=cfg.get('channel', 0),
                corner=corner,
                calibration=cfg.get('calibration', {}),
                simulate=simulate
            )

        logger.info(f"Initialized {len(self.potentiometers)} potentiometers")

    def read_all(self) -> Dict[str, PotentiometerReading]:
        """
        Read all potentiometers.

        Returns:
            Dictionary mapping corner names to readings
        """
        readings = {}
        for corner, pot in self.potentiometers.items():
            readings[corner] = pot.read()
        return readings

    def read_positions(self) -> Dict[str, float]:
        """
        Read position from all corners.

        Returns:
            Dictionary mapping corner names to positions in mm
        """
        positions = {}
        for corner, pot in self.potentiometers.items():
            position, _ = pot.read_position()
            positions[corner] = position
        return positions

    def read_velocities(self) -> Dict[str, float]:
        """
        Read velocities from all corners.

        Returns:
            Dictionary mapping corner names to velocities in mm/s
        """
        readings = self.read_all()
        return {corner: r.velocity_mm_s for corner, r in readings.items()}

    def calculate_body_motion(self) -> Dict[str, float]:
        """
        Calculate body motion from corner displacements.

        Uses the four corner positions to estimate:
        - Heave (average of all corners)
        - Roll (difference between left and right)
        - Pitch (difference between front and rear)
        - Warp (diagonal difference)

        Returns:
            Dictionary with heave_mm, roll_mm, pitch_mm, warp_mm
        """
        positions = self.read_positions()

        fl = positions.get('front_left', 0)
        fr = positions.get('front_right', 0)
        rl = positions.get('rear_left', 0)
        rr = positions.get('rear_right', 0)

        return {
            'heave_mm': (fl + fr + rl + rr) / 4,
            'roll_mm': ((fl + rl) - (fr + rr)) / 2,
            'pitch_mm': ((fl + fr) - (rl + rr)) / 2,
            'warp_mm': ((fl + rr) - (fr + rl)) / 2
        }

    def get_travel_stats(
        self,
        readings: List[Dict[str, PotentiometerReading]]
    ) -> Dict[str, dict]:
        """
        Calculate travel statistics from a series of readings.

        Args:
            readings: List of reading dictionaries

        Returns:
            Statistics for each corner including min, max, mean, std
        """
        stats = {}

        for corner in self.CORNERS:
            positions = [r[corner].position_mm for r in readings if corner in r]

            if positions:
                stats[corner] = {
                    'min_mm': min(positions),
                    'max_mm': max(positions),
                    'mean_mm': np.mean(positions),
                    'std_mm': np.std(positions),
                    'range_mm': max(positions) - min(positions)
                }

        return stats

    def close(self) -> None:
        """Close all ADC connections."""
        for adc in self._adcs.values():
            adc.close()
