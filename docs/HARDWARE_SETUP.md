# Hardware Setup Guide

This guide covers the complete hardware installation for the Celica Suspension DAQ system.

## Components Overview

### Central Unit
- **Raspberry Pi 4 (4GB)** - Main processing unit
- **MicroSD Card (32GB+)** - Storage for OS and data
- **Power supply** - 12V to 5V/3A buck converter

### Sensors
- **5x MPU6050** - 6-axis accelerometer/gyroscope
  - 1x at vehicle CG (body motion)
  - 4x at unsprung mass (wheel motion)
- **4x ADS1115** - 16-bit ADC for potentiometers
- **4x Linear Potentiometer** - 100mm stroke
- **1x TCA9548A** - I2C multiplexer
- **1x NEO-6M GPS** - Position and speed
- **1x ELM327** - OBD-II Bluetooth adapter
- **5x DS18B20** - Temperature sensors

## Mounting Locations

### Body Accelerometer (MPU6050 @ 0x68)
Mount at the vehicle's center of gravity:
- **Location**: Transmission tunnel, near center console
- **Orientation**: X-axis forward, Y-axis left, Z-axis up
- **Mounting**: Double-sided tape or vibration-damping mount
- **Cable routing**: Under carpet to Raspberry Pi

### Corner Accelerometers (via TCA9548A)
Mount on unsprung mass at each corner:
- **Location**: Lower control arm or trailing arm
- **Orientation**: Z-axis vertical (up)
- **Protection**: Weatherproof enclosure required
- **Cable routing**: Along brake lines, secured with zip ties

### Linear Potentiometers
Mount parallel to shock absorbers:
- **Type**: Linear position sensor, 100mm stroke
- **Mounting**: Custom brackets (aluminum or 3D printed)
- **Rod end**: Spherical bearing for misalignment
- **Cable**: Shielded cable recommended

### GPS Module (NEO-6M)
- **Location**: Dashboard or roof (clear sky view)
- **Antenna**: Active ceramic antenna
- **Orientation**: Antenna facing up

### Temperature Sensors (DS18B20)
- **Shock body**: Attach with thermal paste and aluminum tape
- **Ambient**: Engine bay inlet or front grille

## Wiring Instructions

### I2C Bus Wiring
All I2C devices share the same bus:
```
Raspberry Pi GPIO 2 (SDA) ──────┬──── Device SDA
Raspberry Pi GPIO 3 (SCL) ──────┬──── Device SCL
                                │
                          4.7kΩ pull-up to 3.3V
```

### Power Distribution
```
12V Vehicle ──► Buck Converter ──► 5V Rail
                                    │
                    ┌───────────────┼───────────────┐
                    │               │               │
              Raspberry Pi     Sensors         GPS Module
```

### Potentiometer Wiring
Each potentiometer connects to an ADS1115:
```
Potentiometer              ADS1115
    VCC ──────────────────── VDD (5V)
    Signal ───────────────── A0 (or A1/A2/A3)
    GND ──────────────────── GND
```

### Temperature Sensor Wiring (1-Wire)
```
                    4.7kΩ
DS18B20 VDD ───┬────/\/\/────── 3.3V
               │
DS18B20 DQ ────┴──────────────── GPIO 4

DS18B20 GND ──────────────────── GND
```

## Enclosure Requirements

### Main Unit (Raspberry Pi)
- IP65 rated weatherproof enclosure
- Ventilation for heat dissipation
- Cable glands for wire entry
- Mounting: Under passenger seat or trunk

### Sensor Modules
- Conformal coating for moisture protection
- Potted electronics for vibration resistance
- UV-resistant cable ties and loom

## Installation Steps

1. **Plan cable routing** before starting
2. **Install potentiometer brackets** at each corner
3. **Mount corner accelerometers** with weatherproofing
4. **Run cables** to central location
5. **Install main enclosure** with Raspberry Pi
6. **Connect all sensors** following wiring diagram
7. **Mount GPS antenna** with clear sky view
8. **Connect power** from vehicle 12V
9. **Pair ELM327** via Bluetooth
10. **Run sensor test** to verify connections

## Verification Checklist

- [ ] All I2C devices detected (`i2cdetect -y 1`)
- [ ] Potentiometers reading voltage changes
- [ ] Accelerometers showing gravity (1g on Z)
- [ ] GPS acquiring satellites
- [ ] Temperature sensors responding
- [ ] OBD-II connected and reading data
- [ ] Dashboard accessible via browser

## Troubleshooting

### I2C Device Not Found
1. Check wiring continuity
2. Verify pull-up resistors present
3. Check device address (some need configuration)
4. Try reducing bus speed

### Noisy Potentiometer Readings
1. Use shielded cables
2. Add 0.1µF capacitor at ADC input
3. Route away from power wires
4. Check ground connections

### GPS No Fix
1. Ensure clear sky view
2. Check antenna connection
3. Wait up to 5 minutes for cold start
4. Verify antenna is active type

### Temperature Sensor Not Reading
1. Check 4.7kΩ pull-up present
2. Verify GPIO 4 enabled for 1-Wire
3. Check for correct sensor ID
4. Inspect for water damage
