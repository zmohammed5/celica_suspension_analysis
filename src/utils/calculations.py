"""
Signal Processing and Physics Calculations.

This module provides mathematical functions for processing
sensor data, including derivatives, integrals, FFT analysis,
and various suspension-specific calculations.
"""

import logging
from typing import Optional, Tuple, Union

import numpy as np
from scipy import signal, integrate
from scipy.interpolate import interp1d

logger = logging.getLogger(__name__)


def calculate_derivative(
    data: np.ndarray,
    sample_rate_hz: float,
    method: str = 'central',
    smooth: bool = True,
    smooth_window: int = 11
) -> np.ndarray:
    """
    Calculate the time derivative of a signal.

    Args:
        data: Input data array
        sample_rate_hz: Sample rate in Hz
        method: Differentiation method ('forward', 'backward', 'central', 'savgol')
        smooth: Whether to smooth the result
        smooth_window: Smoothing window size

    Returns:
        Derivative array (same length as input)
    """
    dt = 1.0 / sample_rate_hz

    if len(data) < 2:
        return np.zeros_like(data)

    if method == 'forward':
        derivative = np.diff(data, prepend=data[0]) / dt
    elif method == 'backward':
        derivative = np.diff(data, append=data[-1]) / dt
    elif method == 'central':
        derivative = np.gradient(data, dt)
    elif method == 'savgol':
        if len(data) >= smooth_window:
            derivative = signal.savgol_filter(
                data, smooth_window, 3, deriv=1, delta=dt
            )
        else:
            derivative = np.gradient(data, dt)
    else:
        derivative = np.gradient(data, dt)

    if smooth and method != 'savgol' and len(data) >= smooth_window:
        derivative = signal.savgol_filter(derivative, smooth_window, 3)

    return derivative


def calculate_second_derivative(
    data: np.ndarray,
    sample_rate_hz: float,
    smooth_window: int = 11
) -> np.ndarray:
    """
    Calculate the second time derivative (acceleration from position).

    Args:
        data: Input data array
        sample_rate_hz: Sample rate in Hz
        smooth_window: Smoothing window size

    Returns:
        Second derivative array
    """
    dt = 1.0 / sample_rate_hz

    if len(data) < smooth_window:
        return np.gradient(np.gradient(data, dt), dt)

    return signal.savgol_filter(data, smooth_window, 3, deriv=2, delta=dt)


def calculate_integral(
    data: np.ndarray,
    sample_rate_hz: float,
    method: str = 'trapezoid',
    remove_drift: bool = True
) -> np.ndarray:
    """
    Calculate the time integral of a signal.

    Args:
        data: Input data array
        sample_rate_hz: Sample rate in Hz
        method: Integration method ('trapezoid', 'simpson', 'cumsum')
        remove_drift: Remove linear drift from result

    Returns:
        Integrated signal
    """
    dt = 1.0 / sample_rate_hz

    if method == 'trapezoid':
        integral = integrate.cumulative_trapezoid(data, dx=dt, initial=0)
    elif method == 'simpson' and len(data) >= 3:
        # Simpson's rule for odd-length arrays
        integral = np.zeros(len(data))
        for i in range(1, len(data)):
            integral[i] = integrate.simpson(data[:i+1], dx=dt)
    else:
        integral = np.cumsum(data) * dt

    if remove_drift:
        # Remove linear trend (drift)
        x = np.arange(len(integral))
        coeffs = np.polyfit(x, integral, 1)
        trend = np.polyval(coeffs, x)
        integral = integral - trend

    return integral


def calculate_rms(
    data: np.ndarray,
    window_size: Optional[int] = None
) -> Union[float, np.ndarray]:
    """
    Calculate Root Mean Square (RMS) value.

    Args:
        data: Input data array
        window_size: If provided, calculate rolling RMS

    Returns:
        RMS value or array of rolling RMS values
    """
    if window_size is None:
        return np.sqrt(np.mean(data ** 2))
    else:
        # Rolling RMS
        squared = data ** 2
        window = np.ones(window_size) / window_size
        mean_squared = np.convolve(squared, window, mode='same')
        return np.sqrt(mean_squared)


def calculate_fft(
    data: np.ndarray,
    sample_rate_hz: float,
    window: str = 'hanning',
    return_magnitude: bool = True
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate Fast Fourier Transform.

    Args:
        data: Input time-domain data
        sample_rate_hz: Sample rate in Hz
        window: Window function ('hanning', 'hamming', 'blackman', 'none')
        return_magnitude: Return magnitude instead of complex values

    Returns:
        Tuple of (frequencies, magnitudes/complex_values)
    """
    n = len(data)

    # Apply window
    if window == 'hanning':
        w = np.hanning(n)
    elif window == 'hamming':
        w = np.hamming(n)
    elif window == 'blackman':
        w = np.blackman(n)
    else:
        w = np.ones(n)

    windowed_data = data * w

    # Calculate FFT
    fft_values = np.fft.rfft(windowed_data)
    frequencies = np.fft.rfftfreq(n, d=1.0/sample_rate_hz)

    if return_magnitude:
        # Normalize magnitude
        magnitudes = np.abs(fft_values) * 2 / n
        return frequencies, magnitudes
    else:
        return frequencies, fft_values


def calculate_psd(
    data: np.ndarray,
    sample_rate_hz: float,
    window_size: Optional[int] = None,
    overlap_pct: float = 50,
    window: str = 'hanning'
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate Power Spectral Density using Welch's method.

    Args:
        data: Input time-domain data
        sample_rate_hz: Sample rate in Hz
        window_size: Segment size (default: len(data)//8)
        overlap_pct: Overlap percentage between segments
        window: Window function

    Returns:
        Tuple of (frequencies, PSD values)
    """
    if window_size is None:
        window_size = min(len(data), max(256, len(data) // 8))

    noverlap = int(window_size * overlap_pct / 100)

    frequencies, psd = signal.welch(
        data,
        fs=sample_rate_hz,
        window=window,
        nperseg=window_size,
        noverlap=noverlap
    )

    return frequencies, psd


def find_natural_frequency(
    frequencies: np.ndarray,
    psd: np.ndarray,
    freq_range: Tuple[float, float] = (0.5, 5.0)
) -> Tuple[float, float]:
    """
    Find the natural frequency from PSD data.

    Args:
        frequencies: Frequency array from PSD
        psd: Power spectral density values
        freq_range: Frequency range to search (Hz)

    Returns:
        Tuple of (natural_frequency_hz, peak_power)
    """
    # Filter to frequency range
    mask = (frequencies >= freq_range[0]) & (frequencies <= freq_range[1])
    filtered_freqs = frequencies[mask]
    filtered_psd = psd[mask]

    if len(filtered_psd) == 0:
        return 0.0, 0.0

    # Find peak
    peak_idx = np.argmax(filtered_psd)
    natural_freq = filtered_freqs[peak_idx]
    peak_power = filtered_psd[peak_idx]

    return float(natural_freq), float(peak_power)


def calculate_damping_ratio(
    data: np.ndarray,
    sample_rate_hz: float,
    natural_freq_hz: Optional[float] = None
) -> float:
    """
    Estimate damping ratio using log decrement method.

    Args:
        data: Time-domain oscillation data
        sample_rate_hz: Sample rate in Hz
        natural_freq_hz: Known natural frequency (optional)

    Returns:
        Estimated damping ratio (0-1)
    """
    # Remove DC offset
    data = data - np.mean(data)

    # Find peaks
    peaks, _ = signal.find_peaks(data, distance=int(sample_rate_hz / 10))

    if len(peaks) < 3:
        return 0.0

    # Get peak amplitudes
    peak_amplitudes = np.abs(data[peaks])

    # Use first few peaks for estimation
    n_peaks = min(5, len(peak_amplitudes) - 1)

    if n_peaks < 1:
        return 0.0

    # Calculate log decrement
    log_decrements = []
    for i in range(n_peaks):
        if peak_amplitudes[i+1] > 0 and peak_amplitudes[i] > 0:
            delta = np.log(peak_amplitudes[i] / peak_amplitudes[i+1])
            log_decrements.append(delta)

    if not log_decrements:
        return 0.0

    avg_log_decrement = np.mean(log_decrements)

    # Calculate damping ratio
    # zeta = delta / sqrt(4*pi^2 + delta^2)
    damping_ratio = avg_log_decrement / np.sqrt(4 * np.pi**2 + avg_log_decrement**2)

    return float(np.clip(damping_ratio, 0, 1))


def resample_data(
    data: np.ndarray,
    timestamps: np.ndarray,
    target_rate_hz: float,
    method: str = 'linear'
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Resample data to a uniform sample rate.

    Args:
        data: Input data array
        timestamps: Timestamps for each sample
        target_rate_hz: Target sample rate in Hz
        method: Interpolation method ('linear', 'cubic', 'nearest')

    Returns:
        Tuple of (resampled_data, new_timestamps)
    """
    if len(data) < 2:
        return data, timestamps

    # Create uniform time array
    t_start = timestamps[0]
    t_end = timestamps[-1]
    dt = 1.0 / target_rate_hz
    new_timestamps = np.arange(t_start, t_end, dt)

    # Interpolate
    interpolator = interp1d(timestamps, data, kind=method, fill_value='extrapolate')
    resampled_data = interpolator(new_timestamps)

    return resampled_data, new_timestamps


def interpolate_data(
    data: np.ndarray,
    factor: int = 2,
    method: str = 'cubic'
) -> np.ndarray:
    """
    Interpolate data to increase resolution.

    Args:
        data: Input data array
        factor: Interpolation factor (2 = double points)
        method: Interpolation method

    Returns:
        Interpolated data array
    """
    if len(data) < 2:
        return data

    x_old = np.arange(len(data))
    x_new = np.linspace(0, len(data) - 1, len(data) * factor)

    interpolator = interp1d(x_old, data, kind=method)
    return interpolator(x_new)


def calculate_velocity_from_position(
    position: np.ndarray,
    sample_rate_hz: float,
    filter_cutoff_hz: Optional[float] = None
) -> np.ndarray:
    """
    Calculate velocity from position data (damper velocity).

    Args:
        position: Position data in mm
        sample_rate_hz: Sample rate in Hz
        filter_cutoff_hz: Optional low-pass filter cutoff

    Returns:
        Velocity in mm/s
    """
    velocity = calculate_derivative(position, sample_rate_hz, method='savgol')

    if filter_cutoff_hz is not None:
        from .filters import LowPassFilter
        lpf = LowPassFilter(filter_cutoff_hz, sample_rate_hz)
        velocity = lpf.filter(velocity)

    return velocity


def calculate_force_from_acceleration(
    acceleration_g: np.ndarray,
    mass_kg: float,
    gravity: float = 9.81
) -> np.ndarray:
    """
    Calculate force from acceleration (F = ma).

    Args:
        acceleration_g: Acceleration in g units
        mass_kg: Mass in kg
        gravity: Gravitational acceleration (m/s^2)

    Returns:
        Force in Newtons
    """
    acceleration_m_s2 = acceleration_g * gravity
    return mass_kg * acceleration_m_s2


def calculate_weight_transfer(
    lateral_g: float,
    cg_height_m: float,
    track_width_m: float,
    total_weight_n: float
) -> float:
    """
    Calculate lateral weight transfer.

    Args:
        lateral_g: Lateral acceleration in g
        cg_height_m: Center of gravity height in meters
        track_width_m: Track width in meters
        total_weight_n: Total vehicle weight in Newtons

    Returns:
        Weight transfer in Newtons (per axle)
    """
    return (lateral_g * cg_height_m * total_weight_n) / track_width_m


def calculate_roll_angle(
    left_travel_mm: float,
    right_travel_mm: float,
    track_width_mm: float
) -> float:
    """
    Calculate body roll angle from suspension travel.

    Args:
        left_travel_mm: Left side suspension travel
        right_travel_mm: Right side suspension travel
        track_width_mm: Track width in mm

    Returns:
        Roll angle in degrees
    """
    travel_diff = left_travel_mm - right_travel_mm
    return np.degrees(np.arctan(travel_diff / track_width_mm))


def calculate_pitch_angle(
    front_travel_mm: float,
    rear_travel_mm: float,
    wheelbase_mm: float
) -> float:
    """
    Calculate body pitch angle from suspension travel.

    Args:
        front_travel_mm: Average front suspension travel
        rear_travel_mm: Average rear suspension travel
        wheelbase_mm: Wheelbase in mm

    Returns:
        Pitch angle in degrees (positive = nose down)
    """
    travel_diff = front_travel_mm - rear_travel_mm
    return np.degrees(np.arctan(travel_diff / wheelbase_mm))


def apply_motion_ratio(
    wheel_travel: np.ndarray,
    motion_ratio: float
) -> np.ndarray:
    """
    Convert wheel travel to shock travel using motion ratio.

    Args:
        wheel_travel: Wheel travel in mm
        motion_ratio: Motion ratio (shock travel / wheel travel)

    Returns:
        Shock travel in mm
    """
    return wheel_travel * motion_ratio


def calculate_wheel_rate(
    spring_rate_n_mm: float,
    motion_ratio: float
) -> float:
    """
    Calculate wheel rate from spring rate and motion ratio.

    Wheel rate = Spring rate * (motion ratio)^2

    Args:
        spring_rate_n_mm: Spring rate in N/mm
        motion_ratio: Motion ratio

    Returns:
        Wheel rate in N/mm
    """
    return spring_rate_n_mm * (motion_ratio ** 2)


def estimate_natural_frequency(
    wheel_rate_n_mm: float,
    corner_mass_kg: float
) -> float:
    """
    Estimate suspension natural frequency.

    f = (1/2π) * sqrt(k/m)

    Args:
        wheel_rate_n_mm: Wheel rate in N/mm
        corner_mass_kg: Sprung mass at corner in kg

    Returns:
        Natural frequency in Hz
    """
    k = wheel_rate_n_mm * 1000  # Convert to N/m
    return (1 / (2 * np.pi)) * np.sqrt(k / corner_mass_kg)
