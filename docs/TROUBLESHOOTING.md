# Troubleshooting Guide

Common issues and solutions for the Celica Suspension DAQ system.

## Hardware Issues

### I2C Devices Not Detected

**Symptoms:**
- `i2cdetect -y 1` shows no devices
- Sensor test fails

**Solutions:**
1. Check wiring connections
   ```bash
   # Verify I2C is enabled
   sudo raspi-config
   # Interface Options → I2C → Enable
   ```

2. Check pull-up resistors
   - 4.7kΩ between SDA and 3.3V
   - 4.7kΩ between SCL and 3.3V

3. Check power supply
   - Measure 3.3V at sensor
   - Measure 5V for ADCs

4. Reduce I2C bus speed
   ```bash
   # In /boot/config.txt
   dtparam=i2c_arm_baudrate=100000
   ```

5. Check for address conflicts
   - Multiple devices can't share same address

### Noisy Sensor Readings

**Symptoms:**
- Erratic values
- Spikes in data
- Poor signal-to-noise ratio

**Solutions:**
1. Use shielded cables
2. Add bypass capacitors (0.1µF) at sensor power pins
3. Ensure proper grounding (star ground)
4. Route signal cables away from power cables
5. Add software filtering (low-pass filter)

### GPS Not Getting Fix

**Symptoms:**
- Satellites = 0
- Position not updating

**Solutions:**
1. Move antenna to better location (clear sky view)
2. Check antenna cable connection
3. Verify antenna is active type (with LNA)
4. Wait 5+ minutes for cold start
5. Update GPS firmware if available

### Temperature Sensors Not Reading

**Symptoms:**
- Sensor ID not found
- Reading shows 0 or invalid

**Solutions:**
1. Check 1-Wire is enabled
   ```bash
   # In /boot/config.txt
   dtoverlay=w1-gpio
   ```

2. Verify 4.7kΩ pull-up resistor
3. Check for water damage
4. Verify sensor ID matches config
   ```bash
   ls /sys/bus/w1/devices/
   ```

### OBD-II Connection Fails

**Symptoms:**
- ELM327 not connecting
- No data from vehicle

**Solutions:**
1. Verify Bluetooth pairing
   ```bash
   bluetoothctl
   > scan on
   > pair XX:XX:XX:XX:XX:XX
   > trust XX:XX:XX:XX:XX:XX
   ```

2. Create rfcomm device
   ```bash
   sudo rfcomm bind 0 XX:XX:XX:XX:XX:XX
   ```

3. Check vehicle ignition is ON
4. Try different OBD protocols
5. Some vehicles have limited PID support

## Software Issues

### Dashboard Not Loading

**Symptoms:**
- Browser shows connection refused
- Dashboard stuck on "Connecting"

**Solutions:**
1. Check Flask is running
   ```bash
   ps aux | grep python
   ```

2. Check port 5000 is not blocked
   ```bash
   sudo netstat -tlnp | grep 5000
   ```

3. Try accessing from localhost first
4. Check firewall settings
   ```bash
   sudo ufw allow 5000
   ```

### Data Not Recording

**Symptoms:**
- Session file empty
- Sample count stays at 0

**Solutions:**
1. Check write permissions
   ```bash
   ls -la data/sessions/
   ```

2. Check disk space
   ```bash
   df -h
   ```

3. Verify session was started
4. Check for errors in logs
   ```bash
   tail -f logs/daq.log
   ```

### Analysis Errors

**Symptoms:**
- Analysis crashes
- NaN or infinite values

**Solutions:**
1. Check data quality
   - Look for gaps
   - Check for stuck values

2. Verify sample rate is correct
3. Ensure sufficient data length
4. Check for division by zero

### High CPU Usage

**Symptoms:**
- System laggy
- Sample rate drops

**Solutions:**
1. Check for runaway processes
   ```bash
   top
   ```

2. Reduce sample rate
3. Disable unused sensors
4. Close dashboard when not needed
5. Increase swap space if needed

## Calibration Issues

### Potentiometer Range Issues

**Symptoms:**
- Readings max out
- Readings don't go to zero

**Solutions:**
1. Check physical mounting range
2. Verify ADC gain setting
3. Recalibrate at physical limits
4. Check for binding in mechanism

### Accelerometer Drift

**Symptoms:**
- Zero offset changes
- Readings drift over time

**Solutions:**
1. Allow warm-up time (5 min)
2. Recalibrate at operating temperature
3. Check for loose mounting
4. Consider temperature compensation

### GPS Accuracy Issues

**Symptoms:**
- Position jumps around
- Speed doesn't match OBD

**Solutions:**
1. Improve antenna placement
2. Enable SBAS (WAAS/EGNOS)
3. Increase update rate
4. Filter GPS data in software

## Performance Optimization

### Improve Sample Rate

1. Use faster I2C (400kHz)
2. Read only needed channels
3. Use burst reads where possible
4. Reduce logging overhead

### Reduce Latency

1. Use real-time kernel
2. Pin process to CPU core
3. Increase thread priority
4. Use memory-mapped I/O

### Improve Storage Performance

1. Use fast SD card (Class 10+)
2. Write in larger batches
3. Compress old sessions
4. Use external SSD for logging

## Getting Help

If issues persist:

1. Check logs: `logs/daq.log`
2. Run sensor test: `python scripts/test_sensors.py`
3. Enable debug logging
4. Create GitHub issue with:
   - Description of problem
   - Steps to reproduce
   - Relevant log entries
   - Hardware configuration
