"""
Sensor modules for Celica Suspension Analysis System.

This package provides interfaces for all hardware sensors used in the
data acquisition system, including accelerometers, potentiometers,
GPS, OBD-II, and temperature sensors.
"""

from .accelerometer import MPU6050, BodyAccelerometer, CornerAccelerometer
from .potentiometer import Potentiometer, PotentiometerArray
from .gps import GPS, GPSData
from .obd import OBDInterface, OBDData
from .temperature import TemperatureSensor, TemperatureArray
from .multiplexer import TCA9548A

__all__ = [
    'MPU6050',
    'BodyAccelerometer',
    'CornerAccelerometer',
    'Potentiometer',
    'PotentiometerArray',
    'GPS',
    'GPSData',
    'OBDInterface',
    'OBDData',
    'TemperatureSensor',
    'TemperatureArray',
    'TCA9548A',
]
