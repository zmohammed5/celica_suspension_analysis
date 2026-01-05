"""Unit tests for filter modules."""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.utils.filters import (
    LowPassFilter,
    HighPassFilter,
    BandPassFilter,
    SavitzkyGolayFilter,
    MovingAverageFilter,
    MedianFilter,
    FilterChain
)
from src.utils.calculations import (
    calculate_derivative,
    calculate_integral,
    calculate_rms,
    calculate_fft,
    find_natural_frequency
)


class TestLowPassFilter:
    """Tests for low-pass filter."""

    def test_removes_high_frequency(self):
        """Test that high frequencies are attenuated."""
        sample_rate = 100
        lpf = LowPassFilter(cutoff_hz=5, sample_rate_hz=sample_rate)

        # Create signal with low (2Hz) and high (20Hz) components
        t = np.linspace(0, 1, sample_rate)
        low_freq = np.sin(2 * np.pi * 2 * t)
        high_freq = np.sin(2 * np.pi * 20 * t)
        signal = low_freq + high_freq

        filtered = lpf.filter(signal)

        # High frequency should be attenuated
        # Compare power in original vs filtered
        high_freq_power_original = np.sum(high_freq ** 2)
        residual = filtered - low_freq
        high_freq_power_filtered = np.sum(residual ** 2)

        assert high_freq_power_filtered < high_freq_power_original * 0.1


class TestHighPassFilter:
    """Tests for high-pass filter."""

    def test_removes_dc_offset(self):
        """Test that DC offset is removed."""
        sample_rate = 100
        hpf = HighPassFilter(cutoff_hz=1, sample_rate_hz=sample_rate)

        # Signal with DC offset
        t = np.linspace(0, 1, sample_rate)
        signal = 5 + np.sin(2 * np.pi * 5 * t)

        filtered = hpf.filter(signal)

        # Mean should be close to zero
        assert abs(np.mean(filtered)) < 1


class TestBandPassFilter:
    """Tests for band-pass filter."""

    def test_passes_target_frequency(self):
        """Test that target frequency passes through."""
        sample_rate = 100
        bpf = BandPassFilter(
            low_cutoff_hz=3,
            high_cutoff_hz=7,
            sample_rate_hz=sample_rate
        )

        # Create signal at 5 Hz (in band)
        t = np.linspace(0, 1, sample_rate)
        signal = np.sin(2 * np.pi * 5 * t)

        filtered = bpf.filter(signal)

        # Correlation should be high
        correlation = np.corrcoef(signal, filtered)[0, 1]
        assert correlation > 0.8


class TestSavitzkyGolayFilter:
    """Tests for Savitzky-Golay filter."""

    def test_smoothing(self):
        """Test signal smoothing."""
        sgf = SavitzkyGolayFilter(window_size=11, polynomial_order=3)

        # Noisy signal
        t = np.linspace(0, 1, 100)
        clean = np.sin(2 * np.pi * 2 * t)
        noisy = clean + np.random.normal(0, 0.2, 100)

        smoothed = sgf.filter(noisy)

        # Smoothed should be closer to clean than noisy
        error_noisy = np.mean((noisy - clean) ** 2)
        error_smoothed = np.mean((smoothed - clean) ** 2)

        assert error_smoothed < error_noisy

    def test_derivative(self):
        """Test smooth derivative calculation."""
        sgf = SavitzkyGolayFilter(window_size=11, polynomial_order=3)

        # Quadratic: x^2, derivative should be 2x
        x = np.linspace(-1, 1, 100)
        y = x ** 2

        derivative = sgf.smooth_derivative(y, sample_rate_hz=100, order=1)

        # Derivative should be approximately 2x
        # Scale factor: derivative is per sample, need to scale
        expected = 2 * x * 100  # 2x scaled by sample rate

        # Check middle portion (edges are affected)
        np.testing.assert_array_almost_equal(
            derivative[20:80],
            expected[20:80],
            decimal=0
        )


class TestMovingAverageFilter:
    """Tests for moving average filter."""

    def test_averaging(self):
        """Test moving average."""
        maf = MovingAverageFilter(window_size=5)

        # Step function
        signal = np.concatenate([np.zeros(50), np.ones(50)])

        filtered = maf.filter(signal)

        # Should be smoothed around the step
        assert filtered[45] < 0.5
        assert filtered[50] > 0  # Started to rise
        assert filtered[55] > 0.5


class TestMedianFilter:
    """Tests for median filter."""

    def test_spike_removal(self):
        """Test spike removal."""
        mf = MedianFilter(window_size=5)

        # Signal with spike
        signal = np.zeros(100)
        signal[50] = 10  # Spike

        filtered = mf.filter(signal)

        # Spike should be removed
        assert filtered[50] < 1


class TestFilterChain:
    """Tests for filter chain."""

    def test_chain_application(self):
        """Test multiple filters in chain."""
        chain = FilterChain([
            MedianFilter(window_size=5),
            LowPassFilter(cutoff_hz=10, sample_rate_hz=100)
        ])

        # Signal with spike and high frequency
        t = np.linspace(0, 1, 100)
        signal = np.sin(2 * np.pi * 2 * t) + np.sin(2 * np.pi * 30 * t)
        signal[50] = 10  # Spike

        filtered = chain.filter(signal)

        # Both spike and high frequency should be reduced
        assert np.max(filtered) < 5  # Spike reduced
        assert np.std(filtered) < np.std(signal)  # Smoothed


class TestCalculations:
    """Tests for calculation functions."""

    def test_derivative(self):
        """Test derivative calculation."""
        t = np.linspace(0, 1, 100)
        y = t ** 2  # Derivative should be 2t

        dy = calculate_derivative(y, sample_rate_hz=100, method='central')

        # At t=0.5 (sample 50), derivative should be ~1 (2*0.5)
        # Scaled by sample rate: 1 * 100 = 100
        assert abs(dy[50] - 100) < 20

    def test_integral(self):
        """Test integral calculation."""
        # Constant signal
        signal = np.ones(100)

        integral = calculate_integral(signal, sample_rate_hz=100)

        # Integral of 1 from 0 to 1 should be 1
        assert abs(integral[-1] - 1) < 0.1

    def test_rms(self):
        """Test RMS calculation."""
        # Sine wave RMS = amplitude / sqrt(2)
        amplitude = 2
        t = np.linspace(0, 10, 1000)
        signal = amplitude * np.sin(2 * np.pi * t)

        rms = calculate_rms(signal)

        expected = amplitude / np.sqrt(2)
        assert abs(rms - expected) < 0.1

    def test_fft(self):
        """Test FFT calculation."""
        sample_rate = 100
        t = np.linspace(0, 1, sample_rate)
        freq = 10
        signal = np.sin(2 * np.pi * freq * t)

        frequencies, magnitudes = calculate_fft(signal, sample_rate)

        # Peak should be at 10 Hz
        peak_idx = np.argmax(magnitudes)
        peak_freq = frequencies[peak_idx]

        assert abs(peak_freq - freq) < 2

    def test_natural_frequency(self):
        """Test natural frequency detection."""
        sample_rate = 100
        t = np.linspace(0, 1, sample_rate)
        freq = 1.5
        signal = np.sin(2 * np.pi * freq * t)

        frequencies, magnitudes = calculate_fft(signal, sample_rate)
        nat_freq, _ = find_natural_frequency(frequencies, magnitudes, (0.5, 5))

        assert abs(nat_freq - freq) < 0.5
