"""Unit tests for sensor modules."""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.sensors.accelerometer import MPU6050, AccelData, GyroData
from src.sensors.potentiometer import ADS1115, Potentiometer, PotentiometerArray
from src.sensors.gps import GPS, GPSData, NMEAParser
from src.sensors.temperature import TemperatureSensor
from src.sensors.multiplexer import TCA9548A


class TestMPU6050:
    """Tests for MPU6050 accelerometer."""

    def test_initialization(self):
        """Test MPU6050 initializes in simulation mode."""
        mpu = MPU6050(simulate=True)
        assert mpu.simulate is True
        assert mpu.accel_range == 8
        assert mpu.gyro_range == 500

    def test_read_accel(self):
        """Test accelerometer reading."""
        mpu = MPU6050(simulate=True)
        accel = mpu.read_accel()

        assert isinstance(accel, AccelData)
        assert hasattr(accel, 'x')
        assert hasattr(accel, 'y')
        assert hasattr(accel, 'z')
        assert hasattr(accel, 'timestamp')

    def test_read_gyro(self):
        """Test gyroscope reading."""
        mpu = MPU6050(simulate=True)
        gyro = mpu.read_gyro()

        assert isinstance(gyro, GyroData)
        assert hasattr(gyro, 'x')
        assert hasattr(gyro, 'y')
        assert hasattr(gyro, 'z')

    def test_simulated_gravity(self):
        """Test simulated data shows gravity on Z-axis."""
        mpu = MPU6050(simulate=True)
        accel = mpu.read_accel()

        # Z should be approximately 1g
        assert 0.9 < accel.z < 1.1

    def test_calibration_setting(self):
        """Test calibration values can be set."""
        mpu = MPU6050(simulate=True)
        mpu.set_calibration(
            accel_offset=[0.01, -0.02, 0.05],
            accel_scale=[1.01, 0.99, 1.0],
            gyro_offset=[1.0, -0.5, 0.2]
        )

        np.testing.assert_array_almost_equal(
            mpu.accel_offset, [0.01, -0.02, 0.05]
        )


class TestADS1115:
    """Tests for ADS1115 ADC."""

    def test_initialization(self):
        """Test ADC initializes."""
        adc = ADS1115(simulate=True)
        assert adc.simulate is True
        assert adc.gain == 1

    def test_read_voltage(self):
        """Test voltage reading."""
        adc = ADS1115(simulate=True)
        voltage = adc.read_voltage(channel=0)

        assert isinstance(voltage, float)
        assert 0 <= voltage <= 5  # Reasonable range


class TestPotentiometer:
    """Tests for potentiometer readings."""

    def test_initialization(self):
        """Test potentiometer initializes."""
        adc = ADS1115(simulate=True)
        calibration = {
            'voltage_min': 0.25,
            'voltage_max': 4.75,
            'position_min_mm': 0,
            'position_max_mm': 100,
            'zero_offset_mm': 50
        }
        pot = Potentiometer(adc, 0, 'front_left', calibration, simulate=True)

        assert pot.corner == 'front_left'
        assert pot.zero_offset == 50

    def test_read_position(self):
        """Test position reading."""
        adc = ADS1115(simulate=True)
        calibration = {
            'voltage_min': 0.25,
            'voltage_max': 4.75,
            'position_min_mm': 0,
            'position_max_mm': 100,
            'zero_offset_mm': 50
        }
        pot = Potentiometer(adc, 0, 'front_left', calibration, simulate=True)

        position, timestamp = pot.read_position()

        assert isinstance(position, float)
        assert -50 <= position <= 50  # Relative to zero offset


class TestGPS:
    """Tests for GPS module."""

    def test_initialization(self):
        """Test GPS initializes."""
        gps = GPS(simulate=True)
        assert gps.simulate is True

    def test_read_data(self):
        """Test GPS data reading."""
        gps = GPS(simulate=True)
        gps.start()
        data = gps.read()
        gps.stop()

        assert isinstance(data, GPSData)
        assert hasattr(data, 'latitude')
        assert hasattr(data, 'longitude')
        assert hasattr(data, 'speed_mph')

    def test_simulated_coordinates(self):
        """Test simulated GPS returns valid coordinates."""
        gps = GPS(simulate=True)
        gps.start()
        data = gps.read()
        gps.stop()

        assert -90 <= data.latitude <= 90
        assert -180 <= data.longitude <= 180


class TestNMEAParser:
    """Tests for NMEA sentence parsing."""

    def test_parse_coordinate(self):
        """Test coordinate parsing."""
        # 35°20.736'N
        result = NMEAParser.parse_coordinate("3520.736", "N")
        assert abs(result - 35.3456) < 0.001

        # 80°41.352'W
        result = NMEAParser.parse_coordinate("08041.352", "W")
        assert abs(result - (-80.6892)) < 0.001


class TestTemperatureSensor:
    """Tests for DS18B20 temperature sensor."""

    def test_initialization(self):
        """Test sensor initializes."""
        sensor = TemperatureSensor("28-test", "shock_fl", simulate=True)
        assert sensor.simulate is True

    def test_read_temperature(self):
        """Test temperature reading."""
        sensor = TemperatureSensor("28-test", "shock_fl", simulate=True)
        reading = sensor.read()

        assert reading.valid is True
        assert 0 < reading.temperature_f < 250  # Reasonable range
        assert -20 < reading.temperature_c < 120


class TestMultiplexer:
    """Tests for TCA9548A multiplexer."""

    def test_initialization(self):
        """Test multiplexer initializes."""
        mux = TCA9548A(simulate=True)
        assert mux.simulate is True

    def test_select_channel(self):
        """Test channel selection."""
        mux = TCA9548A(simulate=True)

        result = mux.select_channel(0)
        assert result is True
        assert mux.current_channel == 0

        result = mux.select_channel(3)
        assert result is True
        assert mux.current_channel == 3

    def test_invalid_channel(self):
        """Test invalid channel raises error."""
        mux = TCA9548A(simulate=True)

        with pytest.raises(ValueError):
            mux.select_channel(8)

    def test_disable_all(self):
        """Test disabling all channels."""
        mux = TCA9548A(simulate=True)
        mux.select_channel(2)
        mux.disable_all()

        assert mux.current_channel is None
