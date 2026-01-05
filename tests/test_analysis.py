"""Unit tests for analysis modules."""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.analysis.damper_analysis import DamperAnalyzer, DamperCharacteristics
from src.analysis.ride_analysis import RideAnalyzer, RideQualityMetrics
from src.analysis.handling_analysis import HandlingAnalyzer, HandlingMetrics
from src.analysis.corner_analysis import CornerAnalyzer, CornerWeights
from src.analysis.track_analysis import TrackAnalyzer


class TestDamperAnalyzer:
    """Tests for damper analysis."""

    @pytest.fixture
    def analyzer(self):
        return DamperAnalyzer(sample_rate_hz=100)

    def test_calculate_velocity(self, analyzer):
        """Test damper velocity calculation."""
        # Sinusoidal position
        t = np.linspace(0, 1, 100)
        position = 10 * np.sin(2 * np.pi * t)

        velocity = analyzer.calculate_damper_velocity(position)

        assert len(velocity) == len(position)
        # Velocity should be cosine (derivative of sine)
        # Max velocity should be approximately 2*pi*10 = 62.8 mm/s
        assert 50 < np.max(velocity) < 80

    def test_velocity_histogram(self, analyzer):
        """Test velocity histogram generation."""
        velocities = np.random.normal(0, 100, 1000)

        histogram = analyzer.analyze_velocity_histogram(velocities)

        assert histogram.total_samples == 1000
        assert len(histogram.counts) == len(histogram.bins) - 1
        assert abs(np.sum(histogram.percentage) - 100) < 1

    def test_analyze_corner(self, analyzer):
        """Test complete corner analysis."""
        n = 1000
        t = np.linspace(0, 10, n)

        # Generate test data
        position = 20 * np.sin(2 * np.pi * 0.5 * t)  # 0.5 Hz oscillation
        accel = 0.5 * np.sin(2 * np.pi * 0.5 * t + np.pi/4)  # Phase shifted

        result = analyzer.analyze_corner(
            corner='front_left',
            position_mm=position,
            accel_z_g=accel
        )

        assert isinstance(result, DamperCharacteristics)
        assert result.corner == 'front_left'
        assert result.rms_velocity > 0


class TestRideAnalyzer:
    """Tests for ride analysis."""

    @pytest.fixture
    def analyzer(self):
        return RideAnalyzer(sample_rate_hz=100)

    def test_natural_frequency_detection(self, analyzer):
        """Test natural frequency detection."""
        # Generate signal with known frequency
        t = np.linspace(0, 10, 1000)
        freq = 1.5  # Hz
        accel = 0.1 * np.sin(2 * np.pi * freq * t)

        nat_freq, amplitude = analyzer.estimate_natural_frequency(accel)

        assert abs(nat_freq - freq) < 0.5  # Within 0.5 Hz

    def test_comfort_rating(self, analyzer):
        """Test comfort rating calculation."""
        # Low amplitude = comfortable
        accel_low = np.random.normal(0, 0.05, 1000)
        rms_low, rating_low = analyzer.calculate_comfort_metrics(accel_low)

        # High amplitude = uncomfortable
        accel_high = np.random.normal(0, 0.3, 1000)
        rms_high, rating_high = analyzer.calculate_comfort_metrics(accel_high)

        assert rms_low < rms_high

    def test_harshness_detection(self, analyzer):
        """Test harshness event detection."""
        # Create data with spike
        accel = np.zeros(1000)
        accel[500:510] = 3.0  # 3g spike

        events = analyzer.detect_harshness_events(accel, threshold_g=2.0)

        assert len(events) >= 1

    def test_analyze(self, analyzer):
        """Test complete analysis."""
        t = np.linspace(0, 10, 1000)
        accel = 0.1 * np.sin(2 * np.pi * 1.5 * t) + np.random.normal(0, 0.02, 1000)

        metrics = analyzer.analyze(accel, location='body')

        assert isinstance(metrics, RideQualityMetrics)
        assert metrics.natural_frequency_hz > 0
        assert 0 <= metrics.damping_ratio <= 1


class TestHandlingAnalyzer:
    """Tests for handling analysis."""

    @pytest.fixture
    def analyzer(self):
        return HandlingAnalyzer(sample_rate_hz=100)

    def test_body_roll_calculation(self, analyzer):
        """Test body roll calculation."""
        n = 100
        pot_fl = np.ones(n) * 10  # 10mm compression
        pot_fr = np.ones(n) * -10  # 10mm extension
        pot_rl = np.ones(n) * 8
        pot_rr = np.ones(n) * -8

        front_roll, rear_roll = analyzer.calculate_body_roll(
            pot_fl, pot_fr, pot_rl, pot_rr
        )

        assert np.mean(front_roll) > 0  # Should show roll
        assert np.mean(rear_roll) > 0

    def test_roll_gradient(self, analyzer):
        """Test roll gradient calculation."""
        n = 1000
        # Linear relationship: 3 deg roll per g
        lateral_g = np.random.uniform(-1, 1, n)
        roll_deg = 3 * lateral_g + np.random.normal(0, 0.1, n)

        gradient, r2 = analyzer.calculate_roll_gradient(lateral_g, roll_deg)

        assert abs(gradient - 3) < 0.5
        assert r2 > 0.9

    def test_gg_diagram(self, analyzer):
        """Test G-G diagram data generation."""
        lateral_g = np.random.uniform(-1.2, 1.2, 1000)
        longitudinal_g = np.random.uniform(-1.0, 0.5, 1000)

        data = analyzer.generate_gg_diagram_data(lateral_g, longitudinal_g)

        assert 'max_lateral_g' in data
        assert 'max_braking_g' in data
        assert data['max_lateral_g'] > 1.0


class TestCornerAnalyzer:
    """Tests for corner weight analysis."""

    @pytest.fixture
    def analyzer(self):
        return CornerAnalyzer(total_weight_lbs=2500)

    def test_corner_weights(self):
        """Test corner weight container."""
        weights = CornerWeights(
            front_left_lbs=775,
            front_right_lbs=750,
            rear_left_lbs=500,
            rear_right_lbs=475
        )

        assert weights.total_weight_lbs == 2500
        assert 55 < weights.front_weight_pct < 65
        assert 48 < weights.cross_weight_pct < 52

    def test_lateral_load_transfer(self, analyzer):
        """Test lateral load transfer calculation."""
        front_transfer, rear_transfer = analyzer.calculate_lateral_load_transfer(1.0)

        assert front_transfer > 0
        assert rear_transfer > 0
        assert front_transfer + rear_transfer < 2500  # Less than total weight


class TestTrackAnalyzer:
    """Tests for track analysis."""

    def test_haversine_distance(self):
        """Test distance calculation."""
        # Distance from (0, 0) to (0, 1) should be ~111km
        dist = TrackAnalyzer.haversine_distance(0, 0, 0, 1)
        assert 110000 < dist < 112000

    def test_lap_detection(self):
        """Test lap crossing detection."""
        analyzer = TrackAnalyzer(
            start_finish_lat=35.0,
            start_finish_lon=-80.0,
            detection_radius_m=50,
            min_lap_time_sec=10
        )

        # Create track passing through start/finish twice
        n = 300
        timestamps = np.arange(n)

        # Simple oval track
        t = np.linspace(0, 2 * np.pi, n)
        lats = 35.0 + 0.001 * np.sin(t)  # ~111m radius
        lons = -80.0 + 0.001 * np.cos(t)

        crossings = analyzer.detect_lap_crossings(timestamps, lats, lons)

        assert len(crossings) >= 1
