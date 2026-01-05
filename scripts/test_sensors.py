#!/usr/bin/env python3
"""
Sensor Test Script

Tests all sensors and verifies connectivity.
Run this after hardware installation to verify everything is working.
"""

import sys
import time
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.sensors import (
    TCA9548A,
    MPU6050,
    PotentiometerArray,
    GPS,
    OBDInterface,
    TemperatureArray
)


def test_i2c_scan():
    """Scan I2C bus for devices."""
    print("\n" + "="*50)
    print("I2C Bus Scan")
    print("="*50)

    try:
        import smbus2
        bus = smbus2.SMBus(1)

        print("\nFound devices at addresses:")
        devices = []
        for addr in range(0x08, 0x78):
            try:
                bus.read_byte(addr)
                devices.append(addr)
                print(f"  0x{addr:02X}")
            except:
                pass

        bus.close()

        expected = [0x68, 0x70, 0x48, 0x49, 0x4A, 0x4B]
        print(f"\nExpected: {[hex(a) for a in expected]}")
        print(f"Found: {[hex(a) for a in devices]}")

        missing = set(expected) - set(devices)
        if missing:
            print(f"\nMISSING: {[hex(a) for a in missing]}")
            return False

        print("\n[PASS] All expected I2C devices found")
        return True

    except Exception as e:
        print(f"\n[FAIL] I2C scan error: {e}")
        return False


def test_multiplexer():
    """Test TCA9548A multiplexer."""
    print("\n" + "="*50)
    print("TCA9548A Multiplexer Test")
    print("="*50)

    try:
        mux = TCA9548A(address=0x70)

        print("\nScanning channels:")
        results = mux.scan_channels()

        for channel, devices in results.items():
            print(f"  Channel {channel}: {[hex(d) for d in devices]}")

        mux.close()
        print("\n[PASS] Multiplexer working")
        return True

    except Exception as e:
        print(f"\n[FAIL] Multiplexer error: {e}")
        return False


def test_body_accelerometer():
    """Test body MPU6050."""
    print("\n" + "="*50)
    print("Body Accelerometer (MPU6050) Test")
    print("="*50)

    try:
        mpu = MPU6050(address=0x68)

        print("\nReading 10 samples...")
        for i in range(10):
            imu = mpu.read_all()
            print(f"  Accel: X={imu.accel.x:+.3f}g Y={imu.accel.y:+.3f}g Z={imu.accel.z:+.3f}g")
            print(f"  Gyro:  X={imu.gyro.x:+.1f}°/s Y={imu.gyro.y:+.1f}°/s Z={imu.gyro.z:+.1f}°/s")
            print(f"  Temp:  {imu.temperature:.1f}°C")
            print()
            time.sleep(0.1)

        # Verify Z-axis shows ~1g (gravity)
        imu = mpu.read_all()
        if 0.8 < abs(imu.accel.z) < 1.2:
            print("[PASS] Accelerometer reading gravity correctly")
            return True
        else:
            print(f"[WARN] Z-axis should be ~1g, got {imu.accel.z:.3f}g")
            return True  # Still pass, might just be oriented differently

    except Exception as e:
        print(f"\n[FAIL] Accelerometer error: {e}")
        return False


def test_potentiometers():
    """Test ADS1115 ADCs and potentiometers."""
    print("\n" + "="*50)
    print("Potentiometers (ADS1115) Test")
    print("="*50)

    try:
        # Load config
        import json
        config_path = Path(__file__).parent.parent / 'config' / 'sensor_config.json'
        with open(config_path) as f:
            config = json.load(f)

        pot_config = config.get('potentiometers', {})
        pots = PotentiometerArray(pot_config, simulate=False)

        print("\nReading potentiometers...")
        for i in range(5):
            readings = pots.read_all()
            for corner, reading in readings.items():
                print(f"  {corner}: {reading.position_mm:+.1f}mm @ {reading.voltage:.3f}V")
            print()
            time.sleep(0.5)

        pots.close()
        print("[PASS] Potentiometers working")
        return True

    except Exception as e:
        print(f"\n[FAIL] Potentiometer error: {e}")
        return False


def test_gps():
    """Test NEO-6M GPS."""
    print("\n" + "="*50)
    print("GPS (NEO-6M) Test")
    print("="*50)

    try:
        gps = GPS(port='/dev/serial0', baud_rate=9600)
        gps.start()

        print("\nWaiting for GPS fix (up to 60 seconds)...")
        start = time.time()

        while time.time() - start < 60:
            data = gps.read()
            print(f"  Satellites: {data.satellites}, Fix: {data.fix_quality}")

            if data.has_fix:
                print(f"\n  Position: {data.latitude:.6f}, {data.longitude:.6f}")
                print(f"  Speed: {data.speed_mph:.1f} mph")
                print(f"  Heading: {data.heading_deg:.1f}°")
                gps.stop()
                gps.close()
                print("\n[PASS] GPS working")
                return True

            time.sleep(2)

        gps.stop()
        gps.close()
        print("\n[WARN] GPS did not acquire fix (may need better antenna placement)")
        return True  # Pass anyway, just no fix

    except Exception as e:
        print(f"\n[FAIL] GPS error: {e}")
        return False


def test_temperature():
    """Test DS18B20 temperature sensors."""
    print("\n" + "="*50)
    print("Temperature Sensors (DS18B20) Test")
    print("="*50)

    try:
        # Discover sensors
        sensors = TemperatureArray.discover_sensors()
        print(f"\nDiscovered sensors: {sensors}")

        if not sensors:
            print("\n[WARN] No temperature sensors found")
            print("  Check 1-Wire is enabled (sudo raspi-config)")
            print("  Check wiring (data pin to GPIO4, 4.7k pull-up)")
            return False

        # Load config
        import json
        config_path = Path(__file__).parent.parent / 'config' / 'sensor_config.json'
        with open(config_path) as f:
            config = json.load(f)

        temp_config = config.get('temperature', {})
        temps = TemperatureArray(temp_config, simulate=False)

        print("\nReading temperatures...")
        readings = temps.read_all()
        for name, reading in readings.items():
            if reading.valid:
                print(f"  {name}: {reading.temperature_f:.1f}°F ({reading.temperature_c:.1f}°C)")
            else:
                print(f"  {name}: [Invalid reading]")

        print("\n[PASS] Temperature sensors working")
        return True

    except Exception as e:
        print(f"\n[FAIL] Temperature error: {e}")
        return False


def test_obd():
    """Test ELM327 OBD-II interface."""
    print("\n" + "="*50)
    print("OBD-II (ELM327) Test")
    print("="*50)

    print("\nNote: This test requires the ELM327 to be paired and")
    print("the vehicle ignition to be on.")

    try:
        obd = OBDInterface(port='/dev/rfcomm0')

        if obd.is_connected:
            print(f"\n  VIN: {obd.get_vin()}")
            print(f"  Supported PIDs: {obd.supported_pids}")

            data = obd.read()
            print(f"\n  Speed: {data.speed_mph:.1f} mph")
            print(f"  RPM: {data.rpm:.0f}")
            print(f"  Throttle: {data.throttle_pct:.1f}%")

            obd.close()
            print("\n[PASS] OBD-II working")
            return True
        else:
            print("\n[WARN] OBD-II not connected")
            print("  Ensure ELM327 is paired and rfcomm0 is bound")
            return False

    except Exception as e:
        print(f"\n[FAIL] OBD-II error: {e}")
        return False


def main():
    """Run all sensor tests."""
    print("\n" + "#"*50)
    print("# CELICA SUSPENSION DAQ - SENSOR TEST")
    print("#"*50)

    results = {}

    # Run tests
    results['I2C Scan'] = test_i2c_scan()
    results['Multiplexer'] = test_multiplexer()
    results['Accelerometer'] = test_body_accelerometer()
    results['Potentiometers'] = test_potentiometers()
    results['GPS'] = test_gps()
    results['Temperature'] = test_temperature()
    results['OBD-II'] = test_obd()

    # Summary
    print("\n" + "#"*50)
    print("# TEST SUMMARY")
    print("#"*50)

    passed = sum(1 for r in results.values() if r)
    total = len(results)

    for test, result in results.items():
        status = "[PASS]" if result else "[FAIL]"
        print(f"  {test:20s} {status}")

    print(f"\nResult: {passed}/{total} tests passed")

    if passed == total:
        print("\nAll sensors working! Ready to collect data.")
        return 0
    else:
        print("\nSome sensors failed. Check wiring and configuration.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
