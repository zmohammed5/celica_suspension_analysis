"""
Digital Signal Processing Filters.

This module provides various digital filters for processing
sensor data, including low-pass, high-pass, band-pass,
and smoothing filters.
"""

import logging
from abc import ABC, abstractmethod
from typing import List, Optional, Union

import numpy as np
from scipy import signal
from scipy.ndimage import median_filter

logger = logging.getLogger(__name__)


class Filter(ABC):
    """Abstract base class for digital filters."""

    @abstractmethod
    def filter(self, data: np.ndarray) -> np.ndarray:
        """
        Apply filter to data.

        Args:
            data: Input data array

        Returns:
            Filtered data array
        """
        pass

    def __call__(self, data: np.ndarray) -> np.ndarray:
        """Allow filter to be called directly."""
        return self.filter(data)


class LowPassFilter(Filter):
    """
    Butterworth Low-Pass Filter.

    Removes high-frequency noise while preserving low-frequency signals.
    Commonly used for smoothing accelerometer and potentiometer data.
    """

    def __init__(
        self,
        cutoff_hz: float,
        sample_rate_hz: float,
        order: int = 4
    ):
        """
        Initialize low-pass filter.

        Args:
            cutoff_hz: Cutoff frequency in Hz
            sample_rate_hz: Sample rate of the data
            order: Filter order (higher = sharper cutoff)
        """
        self.cutoff_hz = cutoff_hz
        self.sample_rate_hz = sample_rate_hz
        self.order = order

        # Calculate normalized cutoff frequency
        nyquist = sample_rate_hz / 2
        normalized_cutoff = cutoff_hz / nyquist

        # Clamp to valid range
        normalized_cutoff = min(0.99, max(0.01, normalized_cutoff))

        # Design filter
        self.b, self.a = signal.butter(order, normalized_cutoff, btype='low')

        logger.debug(f"LowPassFilter: {cutoff_hz}Hz, order={order}")

    def filter(self, data: np.ndarray) -> np.ndarray:
        """
        Apply low-pass filter.

        Args:
            data: Input data array

        Returns:
            Filtered data array
        """
        if len(data) < 3 * self.order:
            return data

        # Use filtfilt for zero-phase filtering
        return signal.filtfilt(self.b, self.a, data)


class HighPassFilter(Filter):
    """
    Butterworth High-Pass Filter.

    Removes low-frequency components (DC offset, drift).
    Useful for isolating dynamic signals from static offsets.
    """

    def __init__(
        self,
        cutoff_hz: float,
        sample_rate_hz: float,
        order: int = 2
    ):
        """
        Initialize high-pass filter.

        Args:
            cutoff_hz: Cutoff frequency in Hz
            sample_rate_hz: Sample rate of the data
            order: Filter order
        """
        self.cutoff_hz = cutoff_hz
        self.sample_rate_hz = sample_rate_hz
        self.order = order

        nyquist = sample_rate_hz / 2
        normalized_cutoff = cutoff_hz / nyquist
        normalized_cutoff = min(0.99, max(0.01, normalized_cutoff))

        self.b, self.a = signal.butter(order, normalized_cutoff, btype='high')

        logger.debug(f"HighPassFilter: {cutoff_hz}Hz, order={order}")

    def filter(self, data: np.ndarray) -> np.ndarray:
        """Apply high-pass filter."""
        if len(data) < 3 * self.order:
            return data

        return signal.filtfilt(self.b, self.a, data)


class BandPassFilter(Filter):
    """
    Butterworth Band-Pass Filter.

    Passes frequencies within a specified range while
    attenuating frequencies outside the range.
    """

    def __init__(
        self,
        low_cutoff_hz: float,
        high_cutoff_hz: float,
        sample_rate_hz: float,
        order: int = 4
    ):
        """
        Initialize band-pass filter.

        Args:
            low_cutoff_hz: Lower cutoff frequency in Hz
            high_cutoff_hz: Upper cutoff frequency in Hz
            sample_rate_hz: Sample rate of the data
            order: Filter order
        """
        self.low_cutoff_hz = low_cutoff_hz
        self.high_cutoff_hz = high_cutoff_hz
        self.sample_rate_hz = sample_rate_hz
        self.order = order

        nyquist = sample_rate_hz / 2
        low_normalized = low_cutoff_hz / nyquist
        high_normalized = high_cutoff_hz / nyquist

        low_normalized = min(0.99, max(0.01, low_normalized))
        high_normalized = min(0.99, max(0.01, high_normalized))

        self.b, self.a = signal.butter(
            order,
            [low_normalized, high_normalized],
            btype='band'
        )

        logger.debug(f"BandPassFilter: {low_cutoff_hz}-{high_cutoff_hz}Hz")

    def filter(self, data: np.ndarray) -> np.ndarray:
        """Apply band-pass filter."""
        if len(data) < 3 * self.order:
            return data

        return signal.filtfilt(self.b, self.a, data)


class SavitzkyGolayFilter(Filter):
    """
    Savitzky-Golay Smoothing Filter.

    Fits a polynomial to a sliding window of data points.
    Excellent for smoothing data while preserving peaks and valleys.
    Can also compute smooth derivatives.
    """

    def __init__(
        self,
        window_size: int = 11,
        polynomial_order: int = 3,
        derivative: int = 0
    ):
        """
        Initialize Savitzky-Golay filter.

        Args:
            window_size: Size of the smoothing window (must be odd)
            polynomial_order: Order of the fitting polynomial
            derivative: Derivative order (0 = smoothing, 1 = first derivative, etc.)
        """
        # Ensure window size is odd
        if window_size % 2 == 0:
            window_size += 1

        if polynomial_order >= window_size:
            polynomial_order = window_size - 1

        self.window_size = window_size
        self.polynomial_order = polynomial_order
        self.derivative = derivative

        logger.debug(f"SavGolFilter: window={window_size}, poly={polynomial_order}")

    def filter(self, data: np.ndarray) -> np.ndarray:
        """Apply Savitzky-Golay filter."""
        if len(data) < self.window_size:
            return data

        return signal.savgol_filter(
            data,
            self.window_size,
            self.polynomial_order,
            deriv=self.derivative
        )

    def smooth_derivative(
        self,
        data: np.ndarray,
        sample_rate_hz: float,
        order: int = 1
    ) -> np.ndarray:
        """
        Calculate smooth derivative.

        Args:
            data: Input data array
            sample_rate_hz: Sample rate
            order: Derivative order

        Returns:
            Smoothed derivative
        """
        if len(data) < self.window_size:
            return np.zeros_like(data)

        delta = 1.0 / sample_rate_hz
        return signal.savgol_filter(
            data,
            self.window_size,
            self.polynomial_order,
            deriv=order,
            delta=delta
        )


class MovingAverageFilter(Filter):
    """
    Simple Moving Average Filter.

    Computes the average of the last N samples.
    Simple but effective for noise reduction.
    """

    def __init__(self, window_size: int = 5):
        """
        Initialize moving average filter.

        Args:
            window_size: Number of samples to average
        """
        self.window_size = window_size
        self._kernel = np.ones(window_size) / window_size

        logger.debug(f"MovingAverageFilter: window={window_size}")

    def filter(self, data: np.ndarray) -> np.ndarray:
        """Apply moving average filter."""
        if len(data) < self.window_size:
            return data

        return np.convolve(data, self._kernel, mode='same')


class MedianFilter(Filter):
    """
    Median Filter.

    Replaces each value with the median of surrounding values.
    Excellent for removing spikes/outliers while preserving edges.
    """

    def __init__(self, window_size: int = 5):
        """
        Initialize median filter.

        Args:
            window_size: Size of the median window
        """
        self.window_size = window_size

        logger.debug(f"MedianFilter: window={window_size}")

    def filter(self, data: np.ndarray) -> np.ndarray:
        """Apply median filter."""
        if len(data) < self.window_size:
            return data

        return median_filter(data, size=self.window_size)


class ExponentialMovingAverage(Filter):
    """
    Exponential Moving Average (EMA) Filter.

    Gives more weight to recent samples. Useful for
    real-time smoothing with minimal lag.
    """

    def __init__(self, alpha: float = 0.1):
        """
        Initialize EMA filter.

        Args:
            alpha: Smoothing factor (0-1). Higher = less smoothing.
        """
        self.alpha = alpha

        logger.debug(f"EMAFilter: alpha={alpha}")

    def filter(self, data: np.ndarray) -> np.ndarray:
        """Apply exponential moving average."""
        if len(data) == 0:
            return data

        result = np.zeros_like(data)
        result[0] = data[0]

        for i in range(1, len(data)):
            result[i] = self.alpha * data[i] + (1 - self.alpha) * result[i-1]

        return result


class FilterChain(Filter):
    """
    Chain multiple filters together.

    Applies filters in sequence, passing the output of
    each filter to the next.
    """

    def __init__(self, filters: List[Filter]):
        """
        Initialize filter chain.

        Args:
            filters: List of filters to apply in order
        """
        self.filters = filters

    def filter(self, data: np.ndarray) -> np.ndarray:
        """Apply all filters in sequence."""
        result = data
        for f in self.filters:
            result = f.filter(result)
        return result

    def add_filter(self, f: Filter) -> 'FilterChain':
        """Add a filter to the chain."""
        self.filters.append(f)
        return self


class RealTimeFilter:
    """
    Real-time filtering for streaming data.

    Maintains filter state between calls for continuous
    filtering of incoming samples.
    """

    def __init__(self, filter_instance: Filter):
        """
        Initialize real-time filter wrapper.

        Args:
            filter_instance: Filter to use
        """
        self.filter = filter_instance
        self._buffer: List[float] = []
        self._buffer_size = 100

    def process(self, sample: float) -> float:
        """
        Process a single sample.

        Args:
            sample: Input sample

        Returns:
            Filtered output
        """
        self._buffer.append(sample)

        if len(self._buffer) > self._buffer_size:
            self._buffer = self._buffer[-self._buffer_size:]

        if len(self._buffer) < 3:
            return sample

        filtered = self.filter.filter(np.array(self._buffer))
        return filtered[-1]

    def reset(self) -> None:
        """Reset filter state."""
        self._buffer = []


def design_notch_filter(
    notch_freq_hz: float,
    sample_rate_hz: float,
    quality_factor: float = 30.0
) -> tuple:
    """
    Design a notch filter to remove a specific frequency.

    Useful for removing power line interference (50/60 Hz).

    Args:
        notch_freq_hz: Frequency to remove
        sample_rate_hz: Sample rate
        quality_factor: Q factor (higher = narrower notch)

    Returns:
        Tuple of (b, a) filter coefficients
    """
    nyquist = sample_rate_hz / 2
    normalized_freq = notch_freq_hz / nyquist

    b, a = signal.iirnotch(normalized_freq, quality_factor)
    return b, a


def apply_notch_filter(
    data: np.ndarray,
    notch_freq_hz: float,
    sample_rate_hz: float,
    quality_factor: float = 30.0
) -> np.ndarray:
    """
    Apply notch filter to data.

    Args:
        data: Input data array
        notch_freq_hz: Frequency to remove
        sample_rate_hz: Sample rate
        quality_factor: Q factor

    Returns:
        Filtered data
    """
    b, a = design_notch_filter(notch_freq_hz, sample_rate_hz, quality_factor)
    return signal.filtfilt(b, a, data)
