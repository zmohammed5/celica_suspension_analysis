/**
 * Celica Suspension DAQ Dashboard JavaScript
 *
 * Handles real-time data visualization, WebSocket communication,
 * and user interface interactions.
 */

// WebSocket connection
let socket = null;
let isRecording = false;
let recordingStartTime = null;
let sampleCount = 0;

// Chart data buffers
const MAX_POINTS = 500;
const ggData = { x: [], y: [] };
const travelHistory = {
    time: [],
    fl: [], fr: [], rl: [], rr: []
};
const speedHistory = {
    time: [],
    speed: []
};

// Previous values for velocity calculation
let prevTravel = { fl: 0, fr: 0, rl: 0, rr: 0 };
let prevTime = Date.now();

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    initializeSocket();
    initializeCharts();
    setupEventListeners();
    loadSessions();
});

/**
 * Initialize WebSocket connection
 */
function initializeSocket() {
    socket = io();

    socket.on('connect', () => {
        console.log('Connected to server');
        updateConnectionStatus('connected');
    });

    socket.on('disconnect', () => {
        console.log('Disconnected from server');
        updateConnectionStatus('disconnected');
    });

    socket.on('connected', (data) => {
        isRecording = data.recording;
        updateRecordingUI();
    });

    socket.on('data_update', (data) => {
        updateDashboard(data);
    });

    socket.on('recording_started', (data) => {
        isRecording = true;
        recordingStartTime = Date.now();
        sampleCount = 0;
        updateRecordingUI();
    });

    socket.on('recording_stopped', (data) => {
        isRecording = false;
        updateRecordingUI();
        loadSessions();
    });

    socket.on('marker_added', (data) => {
        showNotification(`Marker added: ${data.label}`);
    });
}

/**
 * Initialize Plotly charts
 */
function initializeCharts() {
    // G-G Diagram
    Plotly.newPlot('ggChart', [{
        x: [],
        y: [],
        mode: 'markers',
        type: 'scattergl',
        marker: {
            size: 3,
            color: 'rgba(233, 69, 96, 0.5)'
        }
    }], {
        paper_bgcolor: 'transparent',
        plot_bgcolor: 'transparent',
        xaxis: {
            title: 'Lateral G',
            range: [-1.5, 1.5],
            color: '#a0a0a0',
            gridcolor: '#333'
        },
        yaxis: {
            title: 'Longitudinal G',
            range: [-1.5, 1.5],
            color: '#a0a0a0',
            gridcolor: '#333',
            scaleanchor: 'x'
        },
        margin: { t: 10, r: 10, b: 40, l: 50 }
    }, { responsive: true });

    // Travel History
    Plotly.newPlot('travelChart', [
        { x: [], y: [], name: 'FL', line: { color: '#1f77b4' } },
        { x: [], y: [], name: 'FR', line: { color: '#ff7f0e' } },
        { x: [], y: [], name: 'RL', line: { color: '#2ca02c' } },
        { x: [], y: [], name: 'RR', line: { color: '#d62728' } }
    ], {
        paper_bgcolor: 'transparent',
        plot_bgcolor: 'transparent',
        xaxis: {
            title: 'Time (s)',
            color: '#a0a0a0',
            gridcolor: '#333'
        },
        yaxis: {
            title: 'Travel (mm)',
            color: '#a0a0a0',
            gridcolor: '#333'
        },
        legend: { orientation: 'h', y: 1.1 },
        margin: { t: 30, r: 10, b: 40, l: 50 }
    }, { responsive: true });

    // Speed Chart
    Plotly.newPlot('speedChart', [{
        x: [],
        y: [],
        type: 'scatter',
        line: { color: '#e94560' }
    }], {
        paper_bgcolor: 'transparent',
        plot_bgcolor: 'transparent',
        xaxis: {
            title: 'Time (s)',
            color: '#a0a0a0',
            gridcolor: '#333'
        },
        yaxis: {
            title: 'Speed (mph)',
            color: '#a0a0a0',
            gridcolor: '#333'
        },
        margin: { t: 10, r: 10, b: 40, l: 50 }
    }, { responsive: true });
}

/**
 * Update dashboard with new data
 */
function updateDashboard(data) {
    const now = Date.now();
    const dt = (now - prevTime) / 1000;
    prevTime = now;

    // Update gauges
    document.getElementById('gaugeSpeed').textContent = Math.round(data.gps_speed || 0);
    document.getElementById('gaugeLateralG').textContent = (data.body_accel_y || 0).toFixed(2);
    document.getElementById('gaugeLongG').textContent = (data.body_accel_x || 0).toFixed(2);
    document.getElementById('gaugeRoll').textContent = (data.body_roll || 0).toFixed(1);
    document.getElementById('gaugePitch').textContent = (data.body_pitch || 0).toFixed(1);

    // Update G bars
    updateGBar('gBarLateral', data.body_accel_y || 0, 1.5);
    updateGBar('gBarLong', data.body_accel_x || 0, 1.5, true);

    // Update suspension travel
    updateTravelBar('FL', data.pot_fl || 0, dt);
    updateTravelBar('FR', data.pot_fr || 0, dt);
    updateTravelBar('RL', data.pot_rl || 0, dt);
    updateTravelBar('RR', data.pot_rr || 0, dt);

    // Update temperatures
    document.getElementById('tempFL').textContent = data.temp_fl ? `${data.temp_fl.toFixed(0)}°F` : '--°F';
    document.getElementById('tempFR').textContent = data.temp_fr ? `${data.temp_fr.toFixed(0)}°F` : '--°F';
    document.getElementById('tempRL').textContent = data.temp_rl ? `${data.temp_rl.toFixed(0)}°F` : '--°F';
    document.getElementById('tempRR').textContent = data.temp_rr ? `${data.temp_rr.toFixed(0)}°F` : '--°F';
    document.getElementById('ambientTemp').textContent = data.temp_ambient ? data.temp_ambient.toFixed(0) : '--';

    // Update GPS status
    const gpsStatus = document.getElementById('gpsStatus');
    if (data.gps_lat && data.gps_lon) {
        gpsStatus.textContent = 'GPS: Fix';
        gpsStatus.classList.add('fix');
    } else {
        gpsStatus.textContent = 'GPS: No Fix';
        gpsStatus.classList.remove('fix');
    }

    // Update charts
    updateCharts(data, dt);

    // Update sample count if recording
    if (isRecording) {
        sampleCount++;
        updateSessionInfo();
    }
}

/**
 * Update G-force bar
 */
function updateGBar(id, value, maxG, vertical = false) {
    const bar = document.getElementById(id);
    const percentage = Math.min(Math.abs(value) / maxG * 50, 50);

    if (vertical) {
        bar.style.height = `${percentage}%`;
        bar.style.bottom = value >= 0 ? '50%' : `${50 - percentage}%`;
    } else {
        bar.style.width = `${percentage * 2}%`;
    }
}

/**
 * Update travel bar and velocity
 */
function updateTravelBar(corner, travel, dt) {
    const bar = document.getElementById(`travel${corner}`);
    const value = document.getElementById(`value${corner}`);
    const vel = document.getElementById(`vel${corner}`);

    const maxTravel = 50; // mm from center
    const percentage = Math.min(Math.abs(travel) / maxTravel * 50, 50);

    bar.style.height = `${percentage}%`;
    bar.style.bottom = travel >= 0 ? '50%' : `${50 - percentage}%`;
    bar.style.background = travel >= 0 ? '#e94560' : '#0f4c75';

    value.textContent = `${travel.toFixed(1)} mm`;

    // Calculate velocity
    const prevKey = corner.toLowerCase();
    const velocity = dt > 0 ? (travel - prevTravel[prevKey]) / dt : 0;
    prevTravel[prevKey] = travel;

    vel.textContent = `${Math.round(velocity)} mm/s`;
}

/**
 * Update charts with new data
 */
function updateCharts(data, dt) {
    const time = travelHistory.time.length > 0
        ? travelHistory.time[travelHistory.time.length - 1] + dt
        : 0;

    // G-G Diagram
    ggData.x.push(data.body_accel_y || 0);
    ggData.y.push(data.body_accel_x || 0);

    if (ggData.x.length > MAX_POINTS) {
        ggData.x.shift();
        ggData.y.shift();
    }

    Plotly.update('ggChart', {
        x: [ggData.x],
        y: [ggData.y]
    });

    // Travel History
    travelHistory.time.push(time);
    travelHistory.fl.push(data.pot_fl || 0);
    travelHistory.fr.push(data.pot_fr || 0);
    travelHistory.rl.push(data.pot_rl || 0);
    travelHistory.rr.push(data.pot_rr || 0);

    if (travelHistory.time.length > MAX_POINTS) {
        travelHistory.time.shift();
        travelHistory.fl.shift();
        travelHistory.fr.shift();
        travelHistory.rl.shift();
        travelHistory.rr.shift();
    }

    Plotly.update('travelChart', {
        x: [travelHistory.time, travelHistory.time, travelHistory.time, travelHistory.time],
        y: [travelHistory.fl, travelHistory.fr, travelHistory.rl, travelHistory.rr]
    });

    // Speed History
    speedHistory.time.push(time);
    speedHistory.speed.push(data.gps_speed || 0);

    if (speedHistory.time.length > MAX_POINTS) {
        speedHistory.time.shift();
        speedHistory.speed.shift();
    }

    Plotly.update('speedChart', {
        x: [speedHistory.time],
        y: [speedHistory.speed]
    });
}

/**
 * Update connection status indicator
 */
function updateConnectionStatus(status) {
    const statusEl = document.getElementById('connectionStatus');
    const dot = statusEl.querySelector('.status-dot');
    const text = statusEl.querySelector('.status-text');

    dot.className = 'status-dot ' + status;

    switch (status) {
        case 'connected':
            text.textContent = 'Connected';
            break;
        case 'disconnected':
            text.textContent = 'Disconnected';
            break;
        default:
            text.textContent = 'Connecting...';
    }
}

/**
 * Update recording UI state
 */
function updateRecordingUI() {
    const btnRecord = document.getElementById('btnRecord');
    const btnMarker = document.getElementById('btnMarker');

    if (isRecording) {
        btnRecord.textContent = 'Stop Recording';
        btnRecord.classList.add('recording');
        btnMarker.disabled = false;
    } else {
        btnRecord.textContent = 'Start Recording';
        btnRecord.classList.remove('recording');
        btnMarker.disabled = true;
    }
}

/**
 * Update session info display
 */
function updateSessionInfo() {
    if (!recordingStartTime) return;

    const elapsed = Math.floor((Date.now() - recordingStartTime) / 1000);
    const hours = Math.floor(elapsed / 3600);
    const minutes = Math.floor((elapsed % 3600) / 60);
    const seconds = elapsed % 60;

    document.querySelector('.session-time').textContent =
        `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
    document.querySelector('.sample-count').textContent = `${sampleCount} samples`;
}

/**
 * Load sessions list
 */
async function loadSessions() {
    try {
        const response = await fetch('/api/sessions');
        const data = await response.json();

        const sessionList = document.getElementById('sessionList');
        sessionList.innerHTML = '';

        data.sessions.forEach(session => {
            const item = document.createElement('div');
            item.className = 'session-item';
            item.innerHTML = `
                <div class="session-item-date">${session.start_time || session.session_id}</div>
                <div class="session-item-info">
                    <span>${session.sample_count || 0} samples</span>
                    <span>${formatDuration(session.duration_sec || 0)}</span>
                </div>
            `;
            item.addEventListener('click', () => viewSession(session.session_id));
            sessionList.appendChild(item);
        });
    } catch (error) {
        console.error('Failed to load sessions:', error);
    }
}

/**
 * Format duration in seconds to MM:SS
 */
function formatDuration(seconds) {
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins}:${secs.toString().padStart(2, '0')}`;
}

/**
 * View a specific session
 */
async function viewSession(sessionId) {
    console.log('View session:', sessionId);
    // TODO: Implement session viewer
}

/**
 * Show notification
 */
function showNotification(message) {
    // Simple notification - could be enhanced with toast library
    console.log('Notification:', message);
}

/**
 * Setup event listeners
 */
function setupEventListeners() {
    // Record button
    document.getElementById('btnRecord').addEventListener('click', async () => {
        if (isRecording) {
            await fetch('/api/recording/stop', { method: 'POST' });
        } else {
            await fetch('/api/recording/start', { method: 'POST' });
        }
    });

    // Marker button
    document.getElementById('btnMarker').addEventListener('click', () => {
        document.getElementById('markerModal').classList.add('show');
    });

    // Save marker
    document.getElementById('btnSaveMarker').addEventListener('click', async () => {
        const label = document.getElementById('markerLabel').value || 'Marker';
        const notes = document.getElementById('markerNotes').value;

        await fetch('/api/recording/marker', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ label, notes })
        });

        document.getElementById('markerModal').classList.remove('show');
        document.getElementById('markerLabel').value = '';
        document.getElementById('markerNotes').value = '';
    });

    // Cancel marker
    document.getElementById('btnCancelMarker').addEventListener('click', () => {
        document.getElementById('markerModal').classList.remove('show');
    });

    // Sessions sidebar
    document.getElementById('btnSessions').addEventListener('click', () => {
        document.getElementById('sidebar').classList.add('open');
    });

    document.getElementById('btnCloseSidebar').addEventListener('click', () => {
        document.getElementById('sidebar').classList.remove('open');
    });

    // Update session timer
    setInterval(() => {
        if (isRecording) {
            updateSessionInfo();
        }
    }, 1000);
}
