"""
Flask Web Dashboard for Celica Suspension Analysis.

Provides a real-time web interface for monitoring and analyzing
suspension data during live sessions and reviewing historical data.
"""

import json
import logging
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

from flask import Flask, jsonify, render_template, request, send_from_directory
from flask_socketio import SocketIO, emit

logger = logging.getLogger(__name__)

# Initialize Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY', 'celica-suspension-secret-key')

# Initialize SocketIO
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global state
current_data: Dict[str, Any] = {}
session_manager = None
is_recording = False
data_lock = threading.Lock()


def init_app(session_mgr=None):
    """Initialize the dashboard with optional session manager."""
    global session_manager
    session_manager = session_mgr


# ============== Routes ==============

@app.route('/')
def index():
    """Render main dashboard page."""
    return render_template('index.html')


@app.route('/api/status')
def get_status():
    """Get current system status."""
    return jsonify({
        'connected': True,
        'recording': is_recording,
        'session_id': session_manager.current_session_id if session_manager else None,
        'sample_count': session_manager.sample_count if session_manager else 0,
        'elapsed_time': session_manager.elapsed_time if session_manager else 0,
        'timestamp': datetime.now().isoformat()
    })


@app.route('/api/data')
def get_current_data():
    """Get current sensor data."""
    with data_lock:
        return jsonify(current_data)


@app.route('/api/sessions')
def list_sessions():
    """List all available sessions."""
    if session_manager is None:
        return jsonify({'sessions': []})

    sessions = session_manager.list_sessions()
    return jsonify({'sessions': sessions})


@app.route('/api/sessions/<session_id>')
def get_session(session_id):
    """Get session data by ID."""
    if session_manager is None:
        return jsonify({'error': 'Session manager not initialized'}), 500

    try:
        session_data = session_manager.load_session(session_id)
        return jsonify(session_data)
    except Exception as e:
        return jsonify({'error': str(e)}), 404


@app.route('/api/recording/start', methods=['POST'])
def start_recording():
    """Start a new recording session."""
    global is_recording

    if session_manager is None:
        return jsonify({'error': 'Session manager not initialized'}), 500

    if is_recording:
        return jsonify({'error': 'Already recording'}), 400

    data = request.get_json() or {}

    session_id = session_manager.start_session(
        vehicle=data.get('vehicle', 'Toyota Celica GT-S'),
        driver=data.get('driver', ''),
        track=data.get('track', ''),
        conditions=data.get('conditions', ''),
        setup_notes=data.get('setup_notes', '')
    )

    is_recording = True
    socketio.emit('recording_started', {'session_id': session_id})

    return jsonify({'session_id': session_id, 'status': 'recording'})


@app.route('/api/recording/stop', methods=['POST'])
def stop_recording():
    """Stop the current recording session."""
    global is_recording

    if session_manager is None:
        return jsonify({'error': 'Session manager not initialized'}), 500

    if not is_recording:
        return jsonify({'error': 'Not recording'}), 400

    metadata = session_manager.stop_session()
    is_recording = False

    socketio.emit('recording_stopped', {
        'session_id': metadata.session_id if metadata else None
    })

    return jsonify({
        'status': 'stopped',
        'session_id': metadata.session_id if metadata else None,
        'sample_count': metadata.sample_count if metadata else 0,
        'duration': metadata.duration_sec if metadata else 0
    })


@app.route('/api/recording/marker', methods=['POST'])
def add_marker():
    """Add a marker to the current recording."""
    if session_manager is None or not is_recording:
        return jsonify({'error': 'Not recording'}), 400

    data = request.get_json() or {}
    label = data.get('label', 'Marker')
    notes = data.get('notes', '')

    session_manager.add_marker(label, notes)

    socketio.emit('marker_added', {'label': label, 'notes': notes})

    return jsonify({'status': 'marker_added', 'label': label})


@app.route('/api/config/vehicle')
def get_vehicle_config():
    """Get vehicle configuration."""
    config_path = Path('config/vehicle_config.json')
    if config_path.exists():
        with open(config_path) as f:
            return jsonify(json.load(f))
    return jsonify({'error': 'Config not found'}), 404


@app.route('/api/config/vehicle', methods=['POST'])
def update_vehicle_config():
    """Update vehicle configuration."""
    config_path = Path('config/vehicle_config.json')
    data = request.get_json()

    with open(config_path, 'w') as f:
        json.dump(data, f, indent=2)

    return jsonify({'status': 'updated'})


# ============== WebSocket Events ==============

@socketio.on('connect')
def handle_connect():
    """Handle client connection."""
    logger.info(f"Client connected: {request.sid}")
    emit('connected', {'status': 'connected', 'recording': is_recording})


@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection."""
    logger.info(f"Client disconnected: {request.sid}")


@socketio.on('request_data')
def handle_data_request():
    """Handle request for current data."""
    with data_lock:
        emit('data_update', current_data)


# ============== Data Broadcasting ==============

def update_data(data: Dict[str, Any]):
    """
    Update current data and broadcast to all clients.

    This function is called from the data acquisition loop
    to push new sensor readings to the dashboard.
    """
    global current_data

    with data_lock:
        current_data = {
            'timestamp': time.time(),
            'pot_fl': data.get('pot_fl_mm', 0),
            'pot_fr': data.get('pot_fr_mm', 0),
            'pot_rl': data.get('pot_rl_mm', 0),
            'pot_rr': data.get('pot_rr_mm', 0),
            'body_accel_x': data.get('body_accel_x_g', 0),
            'body_accel_y': data.get('body_accel_y_g', 0),
            'body_accel_z': data.get('body_accel_z_g', 0),
            'body_roll': data.get('body_roll_deg', 0),
            'body_pitch': data.get('body_pitch_deg', 0),
            'yaw_rate': data.get('body_yaw_rate_dps', 0),
            'gps_speed': data.get('gps_speed_mph', 0),
            'gps_lat': data.get('gps_lat', 0),
            'gps_lon': data.get('gps_lon', 0),
            'steering_angle': data.get('steering_angle_deg', 0),
            'throttle': data.get('throttle_pct', 0),
            'brake_pressure': data.get('brake_pressure_psi', 0),
            'temp_fl': data.get('temp_shock_fl_f', 0),
            'temp_fr': data.get('temp_shock_fr_f', 0),
            'temp_rl': data.get('temp_shock_rl_f', 0),
            'temp_rr': data.get('temp_shock_rr_f', 0),
            'temp_ambient': data.get('temp_ambient_f', 0)
        }

    # Broadcast to all connected clients
    socketio.emit('data_update', current_data)


def run_dashboard(host: str = '0.0.0.0', port: int = 5000, debug: bool = False):
    """
    Run the dashboard server.

    Args:
        host: Host address to bind
        port: Port number
        debug: Enable debug mode
    """
    logger.info(f"Starting dashboard on {host}:{port}")
    socketio.run(app, host=host, port=port, debug=debug, use_reloader=False)


# ============== Main ==============

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    run_dashboard(debug=True)
