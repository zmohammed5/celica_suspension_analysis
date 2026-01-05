# Celica Suspension Analysis System

[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-4-red.svg)](https://www.raspberrypi.org/)

A professional-grade suspension data acquisition and analysis system for the Toyota Celica (5th generation, 1990-1993). This system measures suspension behavior in real-time, collecting data from sensors at all four corners to characterize damper performance, body motion, and handling dynamics.

## Features

- **Real-time data acquisition** at 100Hz from 15+ sensor channels
- **Damper analysis** with force-velocity curve estimation
- **Ride quality metrics** including ISO 2631 weighted acceleration
- **Handling dynamics** with roll/pitch gradient calculation
- **GPS track mapping** with lap timing and sector analysis
- **Web-based dashboard** for live monitoring
- **Comprehensive analysis tools** for post-session review

## Hardware Requirements

| Component | Description | Est. Cost |
|-----------|-------------|-----------|
| Raspberry Pi 4 (4GB) | Central data logger | $55 |
| 5x MPU6050 | Accelerometer/gyro modules | $15 |
| TCA9548A | I2C multiplexer | $5 |
| 4x ADS1115 | 16-bit ADC modules | $20 |
| 4x Linear Potentiometer | 100mm stroke, shock travel | $60 |
| NEO-6M GPS | Position and speed | $12 |
| ELM327 Bluetooth | OBD-II interface | $15 |
| 5x DS18B20 | Temperature sensors | $10 |
| Power supply, wiring, enclosure | 12V to 5V, connectors, case | $30 |
| **Total** | | **~$220** |

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 4                          │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                 Main Application                     │   │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐   │   │
│  │  │ Sensors │ │  Data   │ │ Analysis│ │Dashboard│   │   │
│  │  │ Module  │ │ Logger  │ │  Engine │ │  (Web)  │   │   │
│  │  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘   │   │
│  └───────┼───────────┼───────────┼───────────┼─────────┘   │
└──────────┼───────────┼───────────┼───────────┼─────────────┘
           │           │           │           │
    ┌──────┴──────┐    │    ┌──────┴──────┐    │
    │   I2C Bus   │    │    │  CSV/JSON   │    │
    │             │    │    │   Files     │    │
    ▼             ▼    ▼    └─────────────┘    ▼
┌───────┐ ┌───────┐ ┌─────┐               ┌─────────┐
│MPU6050│ │ADS1115│ │GPS  │               │ Browser │
│ x5    │ │ x4    │ │     │               │         │
└───────┘ └───────┘ └─────┘               └─────────┘
```

## I2C Address Map

| Address | Device | Function |
|---------|--------|----------|
| 0x68 | MPU6050 | Body accelerometer (CG) |
| 0x70 | TCA9548A | I2C multiplexer |
| 0x48 | ADS1115 | Front-left potentiometer |
| 0x49 | ADS1115 | Front-right potentiometer |
| 0x4A | ADS1115 | Rear-left potentiometer |
| 0x4B | ADS1115 | Rear-right potentiometer |

## Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/zmohammed5/celica_suspension_analysis.git
cd celica_suspension_analysis
```

### 2. Install Dependencies

```bash
# On Raspberry Pi
sudo ./scripts/setup.sh

# Or manually
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3. Configure Hardware

Edit configuration files in `config/`:
- `vehicle_config.json` - Vehicle specifications
- `sensor_config.json` - Sensor addresses and calibration
- `analysis_config.json` - Analysis parameters

### 4. Run Sensor Test

```bash
python scripts/test_sensors.py
```

### 5. Start the DAQ

```bash
# With dashboard
python src/main.py

# Simulation mode (no hardware)
python src/main.py --simulate
```

Access the dashboard at `http://localhost:5000`

## Data Analysis

### Damper Analysis
```python
from src.analysis import DamperAnalyzer

analyzer = DamperAnalyzer(sample_rate_hz=100)
results = analyzer.analyze_corner(
    corner='front_left',
    position_mm=pot_data,
    accel_z_g=accel_data
)
print(f"Compression coefficient: {results.compression_coefficient:.2f} N/(mm/s)")
```

### Ride Quality
```python
from src.analysis import RideAnalyzer

analyzer = RideAnalyzer(sample_rate_hz=100)
metrics = analyzer.analyze(body_accel_z)
print(f"Natural frequency: {metrics.natural_frequency_hz:.2f} Hz")
print(f"Comfort rating: {metrics.comfort_rating}")
```

### Track Mapping
```python
from src.analysis import TrackAnalyzer

analyzer = TrackAnalyzer(
    start_finish_lat=35.3456,
    start_finish_lon=-80.6892
)
laps = analyzer.extract_laps(timestamps, lats, lons, speeds, lat_g, lon_g)
print(f"Best lap: {laps[0].lap_time_sec:.3f}s")
```

## Wiring Diagram

```
                    Raspberry Pi 4
                    ┌─────────────┐
                    │             │
     ┌──────────────┤ GPIO 2 SDA  ├─────┬─────┬─────┬─────┐
     │              │ GPIO 3 SCL  ├──┬──┼──┬──┼──┬──┼──┬──┤
     │              │             │  │  │  │  │  │  │  │  │
     │   ┌──────────┤ GPIO 4      ├──┼──┼──┼──┼──┼──┼──┼──┼── 1-Wire (DS18B20)
     │   │          │             │  │  │  │  │  │  │  │  │
     │   │   ┌──────┤ GPIO 14 TX  │  │  │  │  │  │  │  │  │
     │   │   │      │ GPIO 15 RX  ├──┼──┼──┼──┼──┼──┼──┼──┼── GPS (NEO-6M)
     │   │   │      │             │  │  │  │  │  │  │  │  │
     │   │   │      │ 3.3V        ├──┴──┴──┴──┴──┴──┴──┴──┘
     │   │   │      │ 5V          ├── Power to sensors
     │   │   │      │ GND         ├── Common ground
     │   │   │      └─────────────┘
     │   │   │
     │   │   └───► GPS NEO-6M
     │   │
     │   └───────► DS18B20 Temperature Sensors (4.7kΩ pull-up to 3.3V)
     │
     └───────────► I2C Bus
                   │
                   ├─► MPU6050 @ 0x68 (Body)
                   │
                   ├─► TCA9548A @ 0x70 (Multiplexer)
                   │   ├─► Ch0: MPU6050 (FL)
                   │   ├─► Ch1: MPU6050 (FR)
                   │   ├─► Ch2: MPU6050 (RL)
                   │   └─► Ch3: MPU6050 (RR)
                   │
                   ├─► ADS1115 @ 0x48 (FL Pot)
                   ├─► ADS1115 @ 0x49 (FR Pot)
                   ├─► ADS1115 @ 0x4A (RL Pot)
                   └─► ADS1115 @ 0x4B (RR Pot)
```

## Data Collected

| Channel | Description | Unit |
|---------|-------------|------|
| pot_fl_mm, pot_fr_mm, pot_rl_mm, pot_rr_mm | Shock displacement | mm |
| accel_fl_z_g, accel_fr_z_g, accel_rl_z_g, accel_rr_z_g | Unsprung mass acceleration | g |
| body_accel_x_g, body_accel_y_g, body_accel_z_g | Body acceleration | g |
| body_roll_deg, body_pitch_deg | Body angles | degrees |
| body_yaw_rate_dps | Yaw rate | deg/s |
| gps_lat, gps_lon | Position | degrees |
| gps_speed_mph | Speed | mph |
| steering_angle_deg | Steering wheel angle | degrees |
| throttle_pct | Throttle position | % |
| temp_shock_*_f | Shock body temperatures | °F |

## Project Structure

```
celica_suspension_analysis/
├── config/                 # Configuration files
├── src/
│   ├── sensors/           # Hardware drivers
│   ├── analysis/          # Analysis modules
│   ├── dashboard/         # Web interface
│   └── utils/             # Utilities
├── scripts/               # Setup and maintenance
├── data/sessions/         # Recorded data
├── docs/                  # Documentation
└── tests/                 # Unit tests
```

## Analysis Capabilities

### Damper Characterization
- Force-velocity curve estimation from F = ma
- Compression/rebound coefficient calculation
- Velocity histogram (time at each damper speed)
- Temperature fade analysis

### Ride Quality
- Natural frequency via FFT analysis
- Damping ratio from log decrement
- ISO 2631 weighted RMS acceleration
- Harshness event detection

### Handling Dynamics
- Roll gradient (deg/g)
- Pitch gradient (deg/g)
- Roll couple distribution
- Understeer gradient estimation
- Transient response metrics

### Track Analysis
- Automatic lap detection
- Sector timing
- G-G diagram generation
- Racing line comparison
- Braking zone analysis

## Safety Warnings

⚠️ **IMPORTANT SAFETY INFORMATION**

- This system is for **off-road/track use only**
- Do not operate the dashboard while driving
- Ensure all sensors and wiring are securely mounted
- Protect electronics from water and debris
- The system may distract the driver - use responsibly
- Data analysis should be performed by qualified personnel
- Suspension modifications should be done by professionals

## Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) for details.

## Acknowledgments

- Toyota for the Celica platform
- The Raspberry Pi Foundation
- The open-source community
