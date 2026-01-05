"""
MPU6050 Accelerometer/Gyroscope Driver.

This module provides interfaces for the MPU6050 6-axis IMU used for
measuring body motion and unsprung mass acceleration at each corner.
"""

import logging
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import smbus2
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False

from .multiplexer import TCA9548A, MultiplexedDevice

logger = logging.getLogger(__name__)


# MPU6050 Register Addresses
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_PWR_MGMT_2 = 0x6C
MPU6050_SMPLRT_DIV = 0x19
MPU6050_CONFIG = 0x1A
MPU6050_GYRO_CONFIG = 0x1B
MPU6050_ACCEL_CONFIG = 0x1C
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_TEMP_OUT_H = 0x41
MPU6050_WHO_AM_I = 0x75

# Scale factors
ACCEL_SCALE = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GYRO_SCALE = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}


@dataclass
class AccelData:
    """Accelerometer data container."""
    x: float  # g
    y: float  # g
    z: float  # g
    timestamp: float

    def magnitude(self) -> float:
        """Calculate total acceleration magnitude."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)


@dataclass
class GyroData:
    """Gyroscope data container."""
    x: float  # deg/s
    y: float  # deg/s
    z: float  # deg/s
    timestamp: float

    def magnitude(self) -> float:
        """Calculate total rotation rate magnitude."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)


@dataclass
class IMUData:
    """Combined IMU data container."""
    accel: AccelData
    gyro: GyroData
    temperature: float  # Celsius

    @property
    def timestamp(self) -> float:
        return self.accel.timestamp


class MPU6050:
    """
    MPU6050 6-axis IMU Driver.

    Provides interface for reading acceleration and gyroscope data
    from the InvenSense MPU6050.

    Attributes:
        address: I2C address (0x68 or 0x69)
        accel_range: Accelerometer full-scale range in g
        gyro_range: Gyroscope full-scale range in deg/s
    """

    def __init__(
        self,
        address: int = 0x68,
        bus_num: int = 1,
        accel_range: int = 8,
        gyro_range: int = 500,
        sample_rate_hz: int = 100,
        dlpf_bandwidth_hz: int = 44,
        simulate: bool = False
    ):
        """
        Initialize the MPU6050.

        Args:
            address: I2C address (0x68 with AD0 low, 0x69 with AD0 high)
            bus_num: I2C bus number
            accel_range: Full-scale range (2, 4, 8, or 16 g)
            gyro_range: Full-scale range (250, 500, 1000, or 2000 deg/s)
            sample_rate_hz: Sample rate in Hz
            dlpf_bandwidth_hz: Digital low-pass filter bandwidth
            simulate: If True, run in simulation mode
        """
        self.address = address
        self.bus_num = bus_num
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self.sample_rate_hz = sample_rate_hz
        self.simulate = simulate or not HAS_SMBUS

        # Calibration values
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        self.accel_scale = np.array([1.0, 1.0, 1.0])
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        self.rotation_matrix = np.eye(3)

        self._bus: Optional['smbus2.SMBus'] = None
        self._accel_scale_factor = ACCEL_SCALE[accel_range]
        self._gyro_scale_factor = GYRO_SCALE[gyro_range]

        # Simulation state
        self._sim_time = 0.0

        if not self.simulate:
            self._init_device()

    def _init_device(self) -> None:
        """Initialize the MPU6050 device."""
        try:
            self._bus = smbus2.SMBus(self.bus_num)

            # Wake up device
            self._write_byte(MPU6050_PWR_MGMT_1, 0x00)
            time.sleep(0.1)

            # Verify device ID
            who_am_i = self._read_byte(MPU6050_WHO_AM_I)
            if who_am_i != 0x68:
                logger.warning(f"Unexpected WHO_AM_I: 0x{who_am_i:02X}")

            # Configure sample rate
            sample_div = max(0, min(255, int(1000 / self.sample_rate_hz) - 1))
            self._write_byte(MPU6050_SMPLRT_DIV, sample_div)

            # Configure DLPF
            dlpf_cfg = self._dlpf_setting()
            self._write_byte(MPU6050_CONFIG, dlpf_cfg)

            # Configure accelerometer range
            accel_cfg = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}[self.accel_range]
            self._write_byte(MPU6050_ACCEL_CONFIG, accel_cfg)

            # Configure gyroscope range
            gyro_cfg = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}[self.gyro_range]
            self._write_byte(MPU6050_GYRO_CONFIG, gyro_cfg)

            logger.info(f"MPU6050 initialized at 0x{self.address:02X}")

        except Exception as e:
            logger.error(f"Failed to initialize MPU6050: {e}")
            self.simulate = True

    def _dlpf_setting(self) -> int:
        """Get DLPF configuration value for desired bandwidth."""
        bandwidths = [260, 184, 94, 44, 21, 10, 5]
        for i, bw in enumerate(bandwidths):
            if bw <= self.sample_rate_hz // 2:
                return i
        return 6

    def _write_byte(self, reg: int, value: int) -> None:
        """Write a byte to a register."""
        if self._bus:
            self._bus.write_byte_data(self.address, reg, value)

    def _read_byte(self, reg: int) -> int:
        """Read a byte from a register."""
        if self._bus:
            return self._bus.read_byte_data(self.address, reg)
        return 0

    def _read_word(self, reg: int) -> int:
        """Read a signed 16-bit word from two consecutive registers."""
        if self._bus:
            high = self._bus.read_byte_data(self.address, reg)
            low = self._bus.read_byte_data(self.address, reg + 1)
            value = (high << 8) | low
            if value >= 0x8000:
                value -= 0x10000
            return value
        return 0

    def _read_block(self, reg: int, length: int) -> list:
        """Read a block of bytes."""
        if self._bus:
            return self._bus.read_i2c_block_data(self.address, reg, length)
        return [0] * length

    def set_calibration(
        self,
        accel_offset: Optional[list] = None,
        accel_scale: Optional[list] = None,
        gyro_offset: Optional[list] = None,
        rotation_matrix: Optional[list] = None
    ) -> None:
        """
        Set calibration parameters.

        Args:
            accel_offset: [x, y, z] offset in g
            accel_scale: [x, y, z] scale factors
            gyro_offset: [x, y, z] offset in deg/s
            rotation_matrix: 3x3 rotation matrix for axis alignment
        """
        if accel_offset is not None:
            self.accel_offset = np.array(accel_offset)
        if accel_scale is not None:
            self.accel_scale = np.array(accel_scale)
        if gyro_offset is not None:
            self.gyro_offset = np.array(gyro_offset)
        if rotation_matrix is not None:
            self.rotation_matrix = np.array(rotation_matrix)

    def read_accel_raw(self) -> Tuple[int, int, int]:
        """Read raw accelerometer values."""
        if self.simulate:
            return self._simulate_accel_raw()

        data = self._read_block(MPU6050_ACCEL_XOUT_H, 6)
        x = (data[0] << 8) | data[1]
        y = (data[2] << 8) | data[3]
        z = (data[4] << 8) | data[5]

        # Convert to signed
        if x >= 0x8000: x -= 0x10000
        if y >= 0x8000: y -= 0x10000
        if z >= 0x8000: z -= 0x10000

        return x, y, z

    def read_gyro_raw(self) -> Tuple[int, int, int]:
        """Read raw gyroscope values."""
        if self.simulate:
            return self._simulate_gyro_raw()

        data = self._read_block(MPU6050_GYRO_XOUT_H, 6)
        x = (data[0] << 8) | data[1]
        y = (data[2] << 8) | data[3]
        z = (data[4] << 8) | data[5]

        # Convert to signed
        if x >= 0x8000: x -= 0x10000
        if y >= 0x8000: y -= 0x10000
        if z >= 0x8000: z -= 0x10000

        return x, y, z

    def read_accel(self) -> AccelData:
        """
        Read calibrated accelerometer data.

        Returns:
            AccelData with x, y, z in g units
        """
        timestamp = time.time()
        raw_x, raw_y, raw_z = self.read_accel_raw()

        # Convert to g and apply calibration
        accel = np.array([
            raw_x / self._accel_scale_factor,
            raw_y / self._accel_scale_factor,
            raw_z / self._accel_scale_factor
        ])

        # Apply offset and scale
        accel = (accel - self.accel_offset) * self.accel_scale

        # Apply rotation matrix for axis alignment
        accel = self.rotation_matrix @ accel

        return AccelData(
            x=float(accel[0]),
            y=float(accel[1]),
            z=float(accel[2]),
            timestamp=timestamp
        )

    def read_gyro(self) -> GyroData:
        """
        Read calibrated gyroscope data.

        Returns:
            GyroData with x, y, z in deg/s
        """
        timestamp = time.time()
        raw_x, raw_y, raw_z = self.read_gyro_raw()

        # Convert to deg/s and apply offset
        gyro = np.array([
            raw_x / self._gyro_scale_factor - self.gyro_offset[0],
            raw_y / self._gyro_scale_factor - self.gyro_offset[1],
            raw_z / self._gyro_scale_factor - self.gyro_offset[2]
        ])

        # Apply rotation matrix
        gyro = self.rotation_matrix @ gyro

        return GyroData(
            x=float(gyro[0]),
            y=float(gyro[1]),
            z=float(gyro[2]),
            timestamp=timestamp
        )

    def read_temperature(self) -> float:
        """
        Read die temperature.

        Returns:
            Temperature in Celsius
        """
        if self.simulate:
            return 25.0 + np.random.normal(0, 0.5)

        raw = self._read_word(MPU6050_TEMP_OUT_H)
        return (raw / 340.0) + 36.53

    def read_all(self) -> IMUData:
        """
        Read all sensor data in a single burst.

        Returns:
            IMUData containing accel, gyro, and temperature
        """
        timestamp = time.time()

        if self.simulate:
            return self._simulate_imu_data(timestamp)

        # Read all 14 bytes in one burst
        data = self._read_block(MPU6050_ACCEL_XOUT_H, 14)

        # Parse accelerometer
        ax = (data[0] << 8) | data[1]
        ay = (data[2] << 8) | data[3]
        az = (data[4] << 8) | data[5]

        # Parse temperature
        temp_raw = (data[6] << 8) | data[7]

        # Parse gyroscope
        gx = (data[8] << 8) | data[9]
        gy = (data[10] << 8) | data[11]
        gz = (data[12] << 8) | data[13]

        # Convert to signed
        for val in [ax, ay, az, temp_raw, gx, gy, gz]:
            if val >= 0x8000:
                val -= 0x10000

        # Convert and calibrate accelerometer
        accel = np.array([
            ax / self._accel_scale_factor,
            ay / self._accel_scale_factor,
            az / self._accel_scale_factor
        ])
        accel = (accel - self.accel_offset) * self.accel_scale
        accel = self.rotation_matrix @ accel

        # Convert and calibrate gyroscope
        gyro = np.array([
            gx / self._gyro_scale_factor - self.gyro_offset[0],
            gy / self._gyro_scale_factor - self.gyro_offset[1],
            gz / self._gyro_scale_factor - self.gyro_offset[2]
        ])
        gyro = self.rotation_matrix @ gyro

        # Convert temperature
        temperature = (temp_raw / 340.0) + 36.53

        return IMUData(
            accel=AccelData(
                x=float(accel[0]),
                y=float(accel[1]),
                z=float(accel[2]),
                timestamp=timestamp
            ),
            gyro=GyroData(
                x=float(gyro[0]),
                y=float(gyro[1]),
                z=float(gyro[2]),
                timestamp=timestamp
            ),
            temperature=temperature
        )

    def _simulate_accel_raw(self) -> Tuple[int, int, int]:
        """Generate simulated accelerometer data."""
        self._sim_time += 0.01

        # Simulate 1g gravity + noise + small vibrations
        x = int(np.random.normal(0, 50))
        y = int(np.random.normal(0, 50))
        z = int(self._accel_scale_factor + np.random.normal(0, 50))

        return x, y, z

    def _simulate_gyro_raw(self) -> Tuple[int, int, int]:
        """Generate simulated gyroscope data."""
        # Small random rotations
        x = int(np.random.normal(0, 20))
        y = int(np.random.normal(0, 20))
        z = int(np.random.normal(0, 20))

        return x, y, z

    def _simulate_imu_data(self, timestamp: float) -> IMUData:
        """Generate complete simulated IMU data."""
        return IMUData(
            accel=AccelData(
                x=np.random.normal(0, 0.02),
                y=np.random.normal(0, 0.02),
                z=1.0 + np.random.normal(0, 0.02),
                timestamp=timestamp
            ),
            gyro=GyroData(
                x=np.random.normal(0, 0.5),
                y=np.random.normal(0, 0.5),
                z=np.random.normal(0, 0.5),
                timestamp=timestamp
            ),
            temperature=25.0 + np.random.normal(0, 0.5)
        )

    def close(self) -> None:
        """Close the I2C bus connection."""
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None


class BodyAccelerometer(MPU6050):
    """
    Specialized MPU6050 for measuring body (sprung mass) motion.

    Mounted at the vehicle's center of gravity to measure:
    - Longitudinal acceleration (braking/acceleration)
    - Lateral acceleration (cornering)
    - Vertical acceleration (heave)
    - Roll, pitch, and yaw rates
    """

    def __init__(self, config: dict, simulate: bool = False):
        """
        Initialize body accelerometer from config.

        Args:
            config: Configuration dictionary from sensor_config.json
            simulate: Run in simulation mode
        """
        super().__init__(
            address=int(config['address'], 16),
            accel_range=config['range_g'],
            gyro_range=config.get('gyro_range_dps', 500),
            sample_rate_hz=config.get('sample_rate_hz', 100),
            simulate=simulate
        )

        # Load calibration
        cal = config.get('calibration', {})
        self.set_calibration(
            accel_offset=cal.get('accel_offset'),
            accel_scale=cal.get('accel_scale'),
            gyro_offset=cal.get('gyro_offset'),
            rotation_matrix=cal.get('rotation_matrix')
        )

    def get_body_motion(self) -> dict:
        """
        Get body motion data in vehicle coordinates.

        Returns:
            Dictionary with:
            - accel_longitudinal_g: Forward/backward acceleration
            - accel_lateral_g: Left/right acceleration
            - accel_vertical_g: Up/down acceleration
            - roll_rate_dps: Roll rate
            - pitch_rate_dps: Pitch rate
            - yaw_rate_dps: Yaw rate
            - timestamp: Measurement timestamp
        """
        imu = self.read_all()

        return {
            'accel_longitudinal_g': imu.accel.x,
            'accel_lateral_g': imu.accel.y,
            'accel_vertical_g': imu.accel.z,
            'roll_rate_dps': imu.gyro.x,
            'pitch_rate_dps': imu.gyro.y,
            'yaw_rate_dps': imu.gyro.z,
            'timestamp': imu.timestamp
        }


class CornerAccelerometer(MultiplexedDevice):
    """
    MPU6050 for measuring unsprung mass acceleration at a wheel corner.

    Connected through a TCA9548A multiplexer to allow multiple
    accelerometers with the same I2C address.
    """

    def __init__(
        self,
        multiplexer: TCA9548A,
        channel: int,
        config: dict,
        corner_name: str,
        simulate: bool = False
    ):
        """
        Initialize corner accelerometer.

        Args:
            multiplexer: TCA9548A multiplexer instance
            channel: Multiplexer channel (0-7)
            config: Configuration dictionary
            corner_name: Corner identifier (e.g., 'front_left')
            simulate: Run in simulation mode
        """
        super().__init__(
            multiplexer=multiplexer,
            channel=channel,
            device_address=int(config['address'], 16)
        )

        self.corner_name = corner_name
        self.simulate = simulate
        self._mpu = MPU6050(
            address=self.device_address,
            accel_range=config.get('range_g', 8),
            simulate=simulate
        )

        # Load calibration
        cal = config.get('calibration', {})
        self._mpu.set_calibration(
            accel_offset=cal.get('accel_offset'),
            accel_scale=cal.get('accel_scale')
        )

    def read_vertical_accel(self) -> Tuple[float, float]:
        """
        Read vertical (Z-axis) acceleration.

        Returns:
            Tuple of (acceleration_g, timestamp)
        """
        self._select_channel()
        accel = self._mpu.read_accel()
        return accel.z, accel.timestamp

    def read_all_axes(self) -> AccelData:
        """
        Read all accelerometer axes.

        Returns:
            AccelData with x, y, z accelerations
        """
        self._select_channel()
        return self._mpu.read_accel()
