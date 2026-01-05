#!/usr/bin/env python3
"""
Celica Suspension Data Acquisition System - Main Application

This is the main entry point for the suspension DAQ system.
It initializes all sensors, starts data collection, and manages
the web dashboard for real-time monitoring.

Usage:
    python main.py                    # Start DAQ with dashboard
    python main.py --no-dashboard     # Start DAQ only
    python main.py --simulate         # Run with simulated sensors
    python main.py --analyze SESSION  # Analyze a recorded session
"""

import argparse
import json
import logging
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.sensors import (
    TCA9548A,
    BodyAccelerometer,
    CornerAccelerometer,
    PotentiometerArray,
    GPS,
    OBDInterface,
    TemperatureArray
)
from src.utils.data_export import SessionManager
from src.utils.calculations import calculate_roll_angle, calculate_pitch_angle
from src.analysis import (
    DamperAnalyzer,
    RideAnalyzer,
    HandlingAnalyzer,
    CornerAnalyzer,
    TrackAnalyzer,
    Visualizer
)

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('logs/daq.log')
    ]
)
logger = logging.getLogger(__name__)


class SuspensionDAQ:
    """
    Main Suspension Data Acquisition System.

    Coordinates all sensors, data logging, and the web dashboard.
    """

    def __init__(
        self,
        config_dir: str = 'config',
        data_dir: str = 'data/sessions',
        simulate: bool = False
    ):
        """
        Initialize the DAQ system.

        Args:
            config_dir: Path to configuration files
            data_dir: Path for session data storage
            simulate: Run with simulated sensors
        """
        self.config_dir = Path(config_dir)
        self.simulate = simulate

        # Load configurations
        self.vehicle_config = self._load_config('vehicle_config.json')
        self.sensor_config = self._load_config('sensor_config.json')
        self.analysis_config = self._load_config('analysis_config.json')

        # Initialize components
        self.session_manager = SessionManager(data_dir)

        # Sensors (initialized in start())
        self.multiplexer: Optional[TCA9548A] = None
        self.body_accel: Optional[BodyAccelerometer] = None
        self.corner_accels: Dict[str, CornerAccelerometer] = {}
        self.potentiometers: Optional[PotentiometerArray] = None
        self.gps: Optional[GPS] = None
        self.obd: Optional[OBDInterface] = None
        self.temperature: Optional[TemperatureArray] = None

        # State
        self._running = False
        self._data_thread: Optional[threading.Thread] = None
        self._dashboard_thread: Optional[threading.Thread] = None
        self._sample_rate = self.sensor_config.get('sampling', {}).get('target_rate_hz', 100)
        self._last_sample_time = 0.0

        logger.info("SuspensionDAQ initialized")

    def _load_config(self, filename: str) -> dict:
        """Load a configuration file."""
        path = self.config_dir / filename
        if path.exists():
            with open(path) as f:
                return json.load(f)
        logger.warning(f"Config file not found: {filename}")
        return {}

    def _init_sensors(self) -> bool:
        """Initialize all sensors."""
        logger.info("Initializing sensors...")
        try:
            # Initialize sensors (simplified for brevity)
            mux_config = self.sensor_config.get('i2c', {}).get('multiplexer', {})
            self.multiplexer = TCA9548A(
                address=int(mux_config.get('address', '0x70'), 16),
                simulate=self.simulate
            )
            
            body_config = self.sensor_config.get('accelerometers', {}).get('body_center', {})
            self.body_accel = BodyAccelerometer(body_config, simulate=self.simulate)
            
            pot_config = self.sensor_config.get('potentiometers', {})
            self.potentiometers = PotentiometerArray(pot_config, simulate=self.simulate)
            
            gps_config = self.sensor_config.get('gps', {})
            self.gps = GPS(simulate=self.simulate)
            self.gps.start()
            
            self.obd = OBDInterface(simulate=self.simulate)
            self.obd.start()
            
            temp_config = self.sensor_config.get('temperature', {})
            self.temperature = TemperatureArray(temp_config, simulate=self.simulate)
            
            logger.info("All sensors initialized")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize sensors: {e}")
            return False

    def _collect_sample(self) -> dict:
        """Collect a single data sample."""
        sample = {'timestamp': time.time()}
        
        # Collect from all sensors
        pot_readings = self.potentiometers.read_all()
        for corner, reading in pot_readings.items():
            key = f"pot_{corner[:2]}_{corner[-1]}_mm"
            sample[key] = reading.position_mm if reading else 0
        
        sample['pot_fl_mm'] = pot_readings.get('front_left', None)
        if sample['pot_fl_mm']:
            sample['pot_fl_mm'] = sample['pot_fl_mm'].position_mm
        sample['pot_fr_mm'] = pot_readings.get('front_right', None) 
        if sample['pot_fr_mm']:
            sample['pot_fr_mm'] = sample['pot_fr_mm'].position_mm
        sample['pot_rl_mm'] = pot_readings.get('rear_left', None)
        if sample['pot_rl_mm']:
            sample['pot_rl_mm'] = sample['pot_rl_mm'].position_mm
        sample['pot_rr_mm'] = pot_readings.get('rear_right', None)
        if sample['pot_rr_mm']:
            sample['pot_rr_mm'] = sample['pot_rr_mm'].position_mm
            
        body = self.body_accel.get_body_motion()
        sample['body_accel_x_g'] = body['accel_longitudinal_g']
        sample['body_accel_y_g'] = body['accel_lateral_g']
        sample['body_accel_z_g'] = body['accel_vertical_g']
        sample['body_yaw_rate_dps'] = body['yaw_rate_dps']
        
        gps = self.gps.read()
        sample['gps_lat'] = gps.latitude
        sample['gps_lon'] = gps.longitude
        sample['gps_speed_mph'] = gps.speed_mph
        
        obd = self.obd.read()
        sample['throttle_pct'] = obd.throttle_pct
        sample['steering_angle_deg'] = obd.steering_angle_deg
        
        temps = self.temperature.read_temperatures_f()
        sample['temp_shock_fl_f'] = temps.get('shock_fl', 0)
        sample['temp_shock_fr_f'] = temps.get('shock_fr', 0)
        sample['temp_ambient_f'] = temps.get('ambient', 0)
        
        return sample

    def _data_loop(self) -> None:
        """Main data collection loop."""
        interval = 1.0 / self._sample_rate
        self._last_sample_time = time.time()
        
        while self._running:
            start = time.time()
            
            sample = self._collect_sample()
            
            if self.session_manager.is_recording:
                self.session_manager.add_sample(sample)
            
            try:
                from src.dashboard.app import update_data
                update_data(sample)
            except:
                pass
            
            elapsed = time.time() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)

    def start(self, with_dashboard: bool = True) -> bool:
        """Start the DAQ system."""
        if self._running:
            return False
        
        if not self._init_sensors():
            return False
        
        self._running = True
        self._data_thread = threading.Thread(target=self._data_loop, daemon=True)
        self._data_thread.start()
        
        if with_dashboard:
            from src.dashboard.app import run_dashboard, init_app
            init_app(self.session_manager)
            self._dashboard_thread = threading.Thread(
                target=run_dashboard,
                kwargs={'host': '0.0.0.0', 'port': 5000},
                daemon=True
            )
            self._dashboard_thread.start()
            logger.info("Dashboard at http://localhost:5000")
        
        logger.info("DAQ started")
        return True

    def stop(self) -> None:
        """Stop the DAQ system."""
        self._running = False
        if self.session_manager.is_recording:
            self.session_manager.stop_session()
        if self.gps: self.gps.stop()
        if self.obd: self.obd.stop()
        logger.info("DAQ stopped")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Celica Suspension DAQ')
    parser.add_argument('--no-dashboard', action='store_true')
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--analyze', type=str)
    args = parser.parse_args()
    
    Path('logs').mkdir(exist_ok=True)
    
    daq = SuspensionDAQ(simulate=args.simulate)
    
    def signal_handler(sig, frame):
        daq.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not daq.start(with_dashboard=not args.no_dashboard):
        sys.exit(1)
    
    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
