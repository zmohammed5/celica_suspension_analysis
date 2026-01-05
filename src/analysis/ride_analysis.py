"""
Ride Quality Analysis Module.

This module provides analysis of vehicle ride quality including:
- Natural frequency calculation via FFT
- Damping ratio estimation
- ISO 2631 weighted acceleration (ride comfort)
- Harshness event detection
"""

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import signal

from ..utils.calculations import (
    calculate_fft,
    calculate_psd,
    find_natural_frequency,
    calculate_damping_ratio,
    calculate_rms
)
from ..utils.filters import BandPassFilter, HighPassFilter

logger = logging.getLogger(__name__)


@dataclass
class RideQualityMetrics:
    """Ride quality metrics for a corner or the body."""
    location: str

    # Natural frequency
    natural_frequency_hz: float = 0.0
    natural_frequency_amplitude: float = 0.0

    # Damping
    damping_ratio: float = 0.0
    damping_quality: str = ""  # "underdamped", "optimal", "overdamped"

    # ISO 2631 weighted acceleration
    weighted_rms_acceleration: float = 0.0
    comfort_rating: str = ""

    # Harshness
    harshness_count: int = 0
    harshness_events: List[Dict] = field(default_factory=list)
    max_harshness_g: float = 0.0

    # Frequency content
    low_freq_energy_pct: float = 0.0  # 0.5-3 Hz (body motion)
    mid_freq_energy_pct: float = 0.0  # 3-10 Hz (ride)
    high_freq_energy_pct: float = 0.0  # 10-30 Hz (harshness)


@dataclass
class FrequencySpectrum:
    """Frequency spectrum data."""
    frequencies: np.ndarray
    magnitudes: np.ndarray
    psd: np.ndarray
    peak_frequency: float
    peak_magnitude: float


# ISO 2631-1 Wk weighting for vertical whole-body vibration
# Simplified frequency response
ISO2631_WK_FREQS = np.array([0.5, 1, 2, 4, 8, 16, 31.5, 63])
ISO2631_WK_WEIGHTS = np.array([0.4, 0.71, 1.0, 1.0, 1.0, 0.5, 0.25, 0.125])


class RideAnalyzer:
    """
    Ride Quality Analyzer.

    Analyzes suspension ride characteristics including natural frequency,
    damping ratio, and comfort metrics based on ISO 2631 standards.
    """

    # Comfort rating thresholds (ISO 2631-1)
    COMFORT_THRESHOLDS = {
        0.315: "Not uncomfortable",
        0.5: "A little uncomfortable",
        0.8: "Fairly uncomfortable",
        1.25: "Uncomfortable",
        2.0: "Very uncomfortable",
        float('inf'): "Extremely uncomfortable"
    }

    # Damping ratio interpretation
    DAMPING_RATINGS = {
        (0, 0.15): "underdamped_light",
        (0.15, 0.25): "underdamped",
        (0.25, 0.35): "optimal",
        (0.35, 0.5): "firm",
        (0.5, 1.0): "overdamped"
    }

    def __init__(
        self,
        sample_rate_hz: float = 100.0,
        fft_window_size: int = 1024,
        expected_natural_freq_range: Tuple[float, float] = (1.0, 3.0)
    ):
        """
        Initialize ride analyzer.

        Args:
            sample_rate_hz: Data sample rate
            fft_window_size: Window size for FFT analysis
            expected_natural_freq_range: Expected natural frequency range (Hz)
        """
        self.sample_rate = sample_rate_hz
        self.fft_window_size = fft_window_size
        self.expected_freq_range = expected_natural_freq_range

        # Filters for frequency band analysis
        self.body_motion_filter = BandPassFilter(0.5, 3.0, sample_rate_hz)
        self.ride_filter = BandPassFilter(3.0, 10.0, sample_rate_hz)
        self.harshness_filter = BandPassFilter(10.0, 30.0, sample_rate_hz)
        self.dc_block = HighPassFilter(0.1, sample_rate_hz)

    def calculate_frequency_spectrum(
        self,
        acceleration_g: np.ndarray
    ) -> FrequencySpectrum:
        """
        Calculate frequency spectrum of acceleration data.

        Args:
            acceleration_g: Acceleration data in g

        Returns:
            FrequencySpectrum with FFT and PSD results
        """
        # Remove DC offset
        accel = acceleration_g - np.mean(acceleration_g)

        # Calculate FFT
        freqs, magnitudes = calculate_fft(accel, self.sample_rate, window='hanning')

        # Calculate PSD
        _, psd = calculate_psd(
            accel,
            self.sample_rate,
            window_size=min(self.fft_window_size, len(accel))
        )

        # Find peak
        peak_freq, peak_mag = find_natural_frequency(
            freqs, magnitudes,
            freq_range=self.expected_freq_range
        )

        return FrequencySpectrum(
            frequencies=freqs,
            magnitudes=magnitudes,
            psd=psd,
            peak_frequency=peak_freq,
            peak_magnitude=peak_mag
        )

    def estimate_natural_frequency(
        self,
        acceleration_g: np.ndarray
    ) -> Tuple[float, float]:
        """
        Estimate natural frequency from acceleration data.

        Args:
            acceleration_g: Vertical acceleration in g

        Returns:
            Tuple of (natural_frequency_hz, amplitude)
        """
        spectrum = self.calculate_frequency_spectrum(acceleration_g)
        return spectrum.peak_frequency, spectrum.peak_magnitude

    def estimate_damping_ratio(
        self,
        acceleration_g: np.ndarray,
        natural_freq_hz: Optional[float] = None
    ) -> float:
        """
        Estimate damping ratio using log decrement method.

        Args:
            acceleration_g: Vertical acceleration data
            natural_freq_hz: Known natural frequency (optional)

        Returns:
            Estimated damping ratio (0-1)
        """
        # Remove DC and high frequency noise
        accel = self.dc_block.filter(acceleration_g)

        # Bandpass around natural frequency if known
        if natural_freq_hz is not None:
            bp = BandPassFilter(
                natural_freq_hz * 0.5,
                natural_freq_hz * 2.0,
                self.sample_rate
            )
            accel = bp.filter(accel)

        return calculate_damping_ratio(accel, self.sample_rate, natural_freq_hz)

    def apply_iso_weighting(
        self,
        acceleration_g: np.ndarray
    ) -> np.ndarray:
        """
        Apply ISO 2631 Wk frequency weighting.

        Args:
            acceleration_g: Raw acceleration data

        Returns:
            Weighted acceleration data
        """
        # Calculate FFT
        freqs, fft_values = calculate_fft(
            acceleration_g, self.sample_rate, return_magnitude=False
        )

        # Interpolate weighting curve
        weights = np.interp(freqs, ISO2631_WK_FREQS, ISO2631_WK_WEIGHTS)

        # Apply weighting in frequency domain
        weighted_fft = fft_values * weights

        # Inverse FFT
        weighted_accel = np.fft.irfft(weighted_fft, n=len(acceleration_g))

        return weighted_accel

    def calculate_comfort_metrics(
        self,
        acceleration_g: np.ndarray
    ) -> Tuple[float, str]:
        """
        Calculate ISO 2631 weighted RMS acceleration and comfort rating.

        Args:
            acceleration_g: Vertical acceleration in g

        Returns:
            Tuple of (weighted_rms_m_s2, comfort_rating_string)
        """
        # Apply ISO weighting
        weighted = self.apply_iso_weighting(acceleration_g)

        # Convert to m/s² and calculate RMS
        weighted_m_s2 = weighted * 9.81
        rms = calculate_rms(weighted_m_s2)

        # Determine comfort rating
        rating = "Extremely uncomfortable"
        for threshold, label in self.COMFORT_THRESHOLDS.items():
            if rms < threshold:
                rating = label
                break

        return rms, rating

    def detect_harshness_events(
        self,
        acceleration_g: np.ndarray,
        threshold_g: float = 2.0,
        min_duration_samples: int = 5
    ) -> List[Dict]:
        """
        Detect harshness events (high-g impulses).

        Args:
            acceleration_g: Acceleration data
            threshold_g: Detection threshold
            min_duration_samples: Minimum event duration

        Returns:
            List of harshness event dictionaries
        """
        events = []

        # Filter to harshness frequencies
        harshness = self.harshness_filter.filter(acceleration_g)

        # Find threshold crossings
        above_threshold = np.abs(harshness) > threshold_g

        # Find event boundaries
        transitions = np.diff(above_threshold.astype(int))
        starts = np.where(transitions == 1)[0]
        ends = np.where(transitions == -1)[0]

        # Match starts and ends
        for start in starts:
            # Find corresponding end
            possible_ends = ends[ends > start]
            if len(possible_ends) > 0:
                end = possible_ends[0]
                duration = end - start

                if duration >= min_duration_samples:
                    peak_idx = start + np.argmax(np.abs(harshness[start:end]))

                    events.append({
                        'start_sample': int(start),
                        'end_sample': int(end),
                        'start_time_sec': start / self.sample_rate,
                        'duration_ms': duration / self.sample_rate * 1000,
                        'peak_g': float(np.abs(harshness[peak_idx])),
                        'peak_sample': int(peak_idx)
                    })

        return events

    def analyze_frequency_bands(
        self,
        acceleration_g: np.ndarray
    ) -> Dict[str, float]:
        """
        Analyze energy distribution across frequency bands.

        Args:
            acceleration_g: Acceleration data

        Returns:
            Dictionary with percentage energy in each band
        """
        # Calculate total energy
        total_energy = np.sum(acceleration_g ** 2)

        if total_energy == 0:
            return {'low': 0, 'mid': 0, 'high': 0}

        # Filter to each band and calculate energy
        body_motion = self.body_motion_filter.filter(acceleration_g)
        ride = self.ride_filter.filter(acceleration_g)
        harshness = self.harshness_filter.filter(acceleration_g)

        low_energy = np.sum(body_motion ** 2) / total_energy * 100
        mid_energy = np.sum(ride ** 2) / total_energy * 100
        high_energy = np.sum(harshness ** 2) / total_energy * 100

        return {
            'low': low_energy,
            'mid': mid_energy,
            'high': high_energy
        }

    def rate_damping(self, damping_ratio: float) -> str:
        """
        Rate damping quality based on damping ratio.

        Args:
            damping_ratio: Measured damping ratio

        Returns:
            Damping quality string
        """
        for (low, high), rating in self.DAMPING_RATINGS.items():
            if low <= damping_ratio < high:
                return rating
        return "unknown"

    def analyze(
        self,
        acceleration_g: np.ndarray,
        location: str = "body"
    ) -> RideQualityMetrics:
        """
        Perform complete ride quality analysis.

        Args:
            acceleration_g: Vertical acceleration data
            location: Location identifier

        Returns:
            RideQualityMetrics with all analysis results
        """
        metrics = RideQualityMetrics(location=location)

        if len(acceleration_g) < self.fft_window_size:
            logger.warning(f"Insufficient data for analysis ({len(acceleration_g)} samples)")
            return metrics

        # Natural frequency
        nat_freq, amplitude = self.estimate_natural_frequency(acceleration_g)
        metrics.natural_frequency_hz = nat_freq
        metrics.natural_frequency_amplitude = amplitude

        # Damping ratio
        damping = self.estimate_damping_ratio(acceleration_g, nat_freq)
        metrics.damping_ratio = damping
        metrics.damping_quality = self.rate_damping(damping)

        # Comfort metrics
        weighted_rms, comfort = self.calculate_comfort_metrics(acceleration_g)
        metrics.weighted_rms_acceleration = weighted_rms
        metrics.comfort_rating = comfort

        # Harshness
        events = self.detect_harshness_events(acceleration_g)
        metrics.harshness_count = len(events)
        metrics.harshness_events = events
        if events:
            metrics.max_harshness_g = max(e['peak_g'] for e in events)

        # Frequency bands
        bands = self.analyze_frequency_bands(acceleration_g)
        metrics.low_freq_energy_pct = bands['low']
        metrics.mid_freq_energy_pct = bands['mid']
        metrics.high_freq_energy_pct = bands['high']

        return metrics

    def analyze_all_corners(
        self,
        corner_data: Dict[str, np.ndarray]
    ) -> Dict[str, RideQualityMetrics]:
        """
        Analyze ride quality at all corners.

        Args:
            corner_data: Dictionary mapping corner names to acceleration arrays

        Returns:
            Dictionary of RideQualityMetrics per corner
        """
        results = {}

        for corner, accel in corner_data.items():
            results[corner] = self.analyze(accel, location=corner)

        return results

    def compare_ride_quality(
        self,
        metrics: Dict[str, RideQualityMetrics]
    ) -> Dict[str, float]:
        """
        Compare ride quality across corners.

        Args:
            metrics: Analysis results for all locations

        Returns:
            Dictionary of comparison metrics
        """
        if len(metrics) < 2:
            return {}

        results = {}

        # Natural frequency spread
        nat_freqs = [m.natural_frequency_hz for m in metrics.values() if m.natural_frequency_hz > 0]
        if nat_freqs:
            results['natural_freq_mean'] = np.mean(nat_freqs)
            results['natural_freq_spread'] = max(nat_freqs) - min(nat_freqs)

        # Damping ratio spread
        dampings = [m.damping_ratio for m in metrics.values() if m.damping_ratio > 0]
        if dampings:
            results['damping_ratio_mean'] = np.mean(dampings)
            results['damping_ratio_spread'] = max(dampings) - min(dampings)

        # Total harshness events
        results['total_harshness_events'] = sum(m.harshness_count for m in metrics.values())

        # Front vs rear comparison
        front_rms = np.mean([
            metrics.get('front_left', RideQualityMetrics('')).weighted_rms_acceleration,
            metrics.get('front_right', RideQualityMetrics('')).weighted_rms_acceleration
        ])
        rear_rms = np.mean([
            metrics.get('rear_left', RideQualityMetrics('')).weighted_rms_acceleration,
            metrics.get('rear_right', RideQualityMetrics('')).weighted_rms_acceleration
        ])

        if rear_rms > 0:
            results['front_rear_rms_ratio'] = front_rms / rear_rms

        return results
