# Quick Start Guide

Get up and running with the Celica Suspension DAQ in 10 minutes.

## 1. Hardware Setup (5 min)

### Minimum for Testing
- Raspberry Pi 4
- 1x MPU6050 (body accelerometer)
- Power supply

### Wiring
```
MPU6050 → Raspberry Pi
  VCC   →   3.3V
  GND   →   GND
  SDA   →   GPIO 2
  SCL   →   GPIO 3
```

## 2. Software Setup (3 min)

```bash
# Clone repository
git clone https://github.com/zmohammed5/celica_suspension_analysis.git
cd celica_suspension_analysis

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

## 3. Test Sensors (1 min)

```bash
# Quick I2C check
i2cdetect -y 1

# Should show device at 0x68
```

## 4. Run in Simulation Mode (1 min)

```bash
python src/main.py --simulate
```

Open browser to `http://localhost:5000`

## 5. Next Steps

1. Add more sensors following [HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md)
2. Calibrate sensors using [SENSOR_CALIBRATION.md](docs/SENSOR_CALIBRATION.md)
3. Mount in vehicle
4. Start recording data!

## Quick Commands

```bash
# Start DAQ with dashboard
python src/main.py

# Start without dashboard
python src/main.py --no-dashboard

# Simulation mode (no hardware)
python src/main.py --simulate

# Analyze a session
python src/main.py --analyze SESSION_ID

# Run sensor test
python scripts/test_sensors.py
```

## Troubleshooting

**I2C not working?**
```bash
sudo raspi-config
# Interface Options → I2C → Enable
sudo reboot
```

**Permission denied?**
```bash
sudo usermod -a -G i2c,dialout $USER
# Log out and back in
```

**Dashboard not loading?**
- Check firewall: `sudo ufw allow 5000`
- Try: `http://127.0.0.1:5000`

For more help, see [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)
