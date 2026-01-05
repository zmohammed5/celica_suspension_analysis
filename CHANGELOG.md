# Changelog

All notable changes to the Celica Suspension Analysis project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-01-05

### Added
- Initial release of Celica Suspension DAQ system
- Sensor drivers for MPU6050, ADS1115, NEO-6M GPS, DS18B20
- I2C multiplexer support (TCA9548A)
- Real-time web dashboard with WebSocket updates
- Damper analysis with force-velocity estimation
- Ride quality analysis with ISO 2631 metrics
- Handling dynamics analysis (roll/pitch gradients)
- Corner weight and load transfer analysis
- GPS track mapping and lap timing
- Session comparison tools
- Comprehensive calibration wizards
- Full documentation suite
- Unit test coverage

### Hardware Support
- Raspberry Pi 4 (4GB recommended)
- 5x MPU6050 accelerometers
- 4x ADS1115 16-bit ADCs
- 4x 100mm linear potentiometers
- NEO-6M GPS module
- ELM327 OBD-II Bluetooth adapter
- 5x DS18B20 temperature sensors

### Analysis Capabilities
- Damper force-velocity curves
- Damper velocity histograms
- Temperature fade analysis
- Natural frequency calculation
- Damping ratio estimation
- Roll/pitch gradient measurement
- Understeer gradient estimation
- Transient response metrics
- Corner weight analysis
- Dynamic load transfer
- Lap time analysis
- Sector comparison
- G-G diagram generation

## [Unreleased]

### Planned
- Support for additional accelerometer types
- CAN bus direct integration
- Machine learning for setup optimization
- Mobile app for dashboard
- Cloud data sync
- Live telemetry streaming
