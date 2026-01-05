# Sensor Calibration Guide

Proper calibration is essential for accurate data. This guide covers calibration procedures for all sensors.

## Potentiometer Calibration

### Equipment Needed
- Ruler or caliper (mm)
- Stable surface
- Notepad

### Procedure

1. **Prepare the potentiometer**
   - Remove from vehicle or use test setup
   - Connect to DAQ system
   - Start calibration wizard: `python -c "from src.utils.calibration import PotentiometerCalibrator; ..."`

2. **Measure at known positions**
   ```
   Position 1: Fully compressed (0mm)
   Position 2: 25mm extended
   Position 3: 50mm extended (mid-stroke)
   Position 4: 75mm extended
   Position 5: Fully extended (100mm)
   ```

3. **Record voltage at each position**
   ```python
   # Example output
   Position  Voltage
   0mm       0.25V
   25mm      1.38V
   50mm      2.50V
   75mm      3.63V
   100mm     4.75V
   ```

4. **Calculate calibration factors**
   - Scale: (100mm - 0mm) / (4.75V - 0.25V) = 22.22 mm/V
   - Offset: Voltage at desired zero point

5. **Update configuration**
   Edit `config/sensor_config.json`:
   ```json
   "calibration": {
     "voltage_min": 0.25,
     "voltage_max": 4.75,
     "position_min_mm": 0,
     "position_max_mm": 100,
     "zero_offset_mm": 52.5
   }
   ```

### Verification
- Move potentiometer through full range
- Verify readings match physical measurements
- Check for linearity (no dead spots)

## Accelerometer Calibration

### Static (Quick) Calibration

1. **Place sensor on level surface**
   - Z-axis pointing up
   - Surface must be truly level

2. **Record readings**
   ```
   Expected: X=0g, Y=0g, Z=1g
   Measured: X=0.012g, Y=-0.008g, Z=1.045g
   ```

3. **Calculate offsets**
   ```
   X_offset = 0.012g
   Y_offset = -0.008g
   Z_offset = 1.045g - 1.0g = 0.045g
   ```

### Six-Position Calibration (More Accurate)

1. **Position the sensor in 6 orientations**
   | Position | Axis | Expected |
   |----------|------|----------|
   | Z-up     | Z    | +1g      |
   | Z-down   | Z    | -1g      |
   | X-up     | X    | +1g      |
   | X-down   | X    | -1g      |
   | Y-up     | Y    | +1g      |
   | Y-down   | Y    | -1g      |

2. **Record readings at each position**

3. **Calculate offset and scale for each axis**
   ```
   Offset = (reading_up + reading_down) / 2
   Scale = 2.0 / (reading_up - reading_down)
   ```

4. **Update configuration**
   ```json
   "calibration": {
     "accel_offset": [0.012, -0.008, 0.045],
     "accel_scale": [1.002, 0.998, 1.001]
   }
   ```

### Gyroscope Zero Offset

1. **Keep sensor completely stationary** for 10 seconds
2. **Record average gyro readings**
3. **These are the zero offsets**
   ```json
   "gyro_offset": [1.25, -0.85, 0.42]
   ```

## Axis Alignment Calibration

For the body accelerometer, align axes with vehicle:
- X = Forward (positive = acceleration)
- Y = Left (positive = left turn)
- Z = Up (positive = bump)

If mounted at an angle, calculate rotation matrix:
```python
# Example: Sensor rotated 5° around Z
import numpy as np
theta = np.radians(5)
rotation_matrix = [
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta), np.cos(theta), 0],
    [0, 0, 1]
]
```

## Temperature Sensor Mapping

1. **Discover all connected sensors**
   ```bash
   python -c "from src.sensors.temperature import TemperatureArray; print(TemperatureArray.discover_sensors())"
   ```

2. **Identify each sensor**
   - Heat sensor location one at a time
   - Note which ID shows temperature increase

3. **Update configuration**
   ```json
   "sensors": {
     "shock_fl": {"id": "28-0316a2795cff"},
     "shock_fr": {"id": "28-0316a27a45ff"},
     ...
   }
   ```

## GPS Configuration

The NEO-6M is factory calibrated. Configure update rate:

```python
# Set to 10Hz update rate
# This is done automatically in the GPS driver
```

## Steering Angle Calibration

1. **Point wheels straight ahead**
2. **Record steering sensor value** (this is the center)
3. **Turn wheel lock-to-lock** and record min/max

```json
"steering_angle": {
  "center_raw": 2048,
  "scale": 0.1,
  "max_angle_deg": 450
}
```

## Static Ride Height

After all calibrations:

1. **Park vehicle on level ground**
2. **Record potentiometer readings** at each corner
3. **These become the "zero" references**

```json
"zero_offset_mm": 52.5  // mm from full compression
```

## Verification Checklist

- [ ] Potentiometers read 0mm at static ride height
- [ ] Accelerometers read 0g lateral/longitudinal when stationary
- [ ] Accelerometers read 1g vertical when stationary
- [ ] Gyros read 0 deg/s when stationary
- [ ] Temperature sensors show reasonable ambient temperature
- [ ] GPS speed matches OBD speed
- [ ] Steering angle reads 0 when straight

## Periodic Recalibration

Recalibrate when:
- Sensors are remounted
- After significant temperature changes
- Readings seem inaccurate
- Before important track sessions
