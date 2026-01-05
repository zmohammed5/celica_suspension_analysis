"""
Handling Dynamics Analysis Module.

This module provides analysis of vehicle handling characteristics including:
- Roll gradient (degrees per g lateral)
- Pitch gradient (degrees per g longitudinal)
- Roll couple distribution
- Understeer/oversteer gradient
- Transient response metrics
"""

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import stats, signal

from ..utils.filters import LowPassFilter

logger = logging.getLogger(__name__)


@dataclass
class HandlingMetrics:
    """Vehicle handling characteristics."""

    # Roll behavior
    roll_gradient_deg_per_g: float = 0.0
    roll_gradient_r_squared: float = 0.0
    max_roll_deg: float = 0.0
    roll_stiffness_front_pct: float = 50.0
    roll_stiffness_rear_pct: float = 50.0

    # Pitch behavior
    pitch_gradient_deg_per_g: float = 0.0
    pitch_gradient_r_squared: float = 0.0
    max_pitch_deg: float = 0.0
    anti_dive_pct: float = 0.0
    anti_squat_pct: float = 0.0

    # Balance
    understeer_gradient_deg_per_g: float = 0.0
    balance_rating: str = ""  # "understeer", "neutral", "oversteer"
    limit_behavior: str = ""

    # Transient response
    roll_time_constant_sec: float = 0.0
    roll_settling_time_sec: float = 0.0
    roll_overshoot_pct: float = 0.0
    yaw_response_time_sec: float = 0.0


@dataclass
class TransientEvent:
    """Single transient maneuver event."""
    start_time: float
    end_time: float
    lateral_g_step: float
    roll_response_time: float
    roll_overshoot: float
    settling_time: float
    steady_state_roll: float


class HandlingAnalyzer:
    """
    Vehicle Handling Analyzer.

    Analyzes handling dynamics including roll/pitch behavior,
    balance characteristics, and transient response.
    """

    def __init__(
        self,
        sample_rate_hz: float = 100.0,
        wheelbase_m: float = 2.6,
        track_width_front_m: float = 1.52,
        track_width_rear_m: float = 1.51,
        cg_height_m: float = 0.47
    ):
        """
        Initialize handling analyzer.

        Args:
            sample_rate_hz: Data sample rate
            wheelbase_m: Vehicle wheelbase in meters
            track_width_front_m: Front track width
            track_width_rear_m: Rear track width
            cg_height_m: CG height in meters
        """
        self.sample_rate = sample_rate_hz
        self.wheelbase = wheelbase_m
        self.track_front = track_width_front_m
        self.track_rear = track_width_rear_m
        self.cg_height = cg_height_m

        self.lpf = LowPassFilter(5.0, sample_rate_hz)

    def calculate_body_roll(
        self,
        pot_fl: np.ndarray,
        pot_fr: np.ndarray,
        pot_rl: np.ndarray,
        pot_rr: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate body roll angle from suspension travel.

        Args:
            pot_fl: Front left potentiometer data (mm)
            pot_fr: Front right potentiometer data (mm)
            pot_rl: Rear left potentiometer data (mm)
            pot_rr: Rear right potentiometer data (mm)

        Returns:
            Tuple of (front_roll_deg, rear_roll_deg)
        """
        # Roll = arctan((left - right) / track_width)
        front_diff = pot_fl - pot_fr
        rear_diff = pot_rl - pot_rr

        front_roll = np.degrees(np.arctan(front_diff / 1000 / self.track_front))
        rear_roll = np.degrees(np.arctan(rear_diff / 1000 / self.track_rear))

        return front_roll, rear_roll

    def calculate_body_pitch(
        self,
        pot_fl: np.ndarray,
        pot_fr: np.ndarray,
        pot_rl: np.ndarray,
        pot_rr: np.ndarray
    ) -> np.ndarray:
        """
        Calculate body pitch angle from suspension travel.

        Args:
            Potentiometer data for all four corners

        Returns:
            Pitch angle in degrees (positive = nose down)
        """
        front_avg = (pot_fl + pot_fr) / 2
        rear_avg = (pot_rl + pot_rr) / 2

        pitch = np.degrees(np.arctan((front_avg - rear_avg) / 1000 / self.wheelbase))

        return pitch

    def calculate_roll_gradient(
        self,
        lateral_g: np.ndarray,
        roll_deg: np.ndarray,
        steady_state_threshold: float = 5.0
    ) -> Tuple[float, float]:
        """
        Calculate roll gradient (degrees per g lateral acceleration).

        Args:
            lateral_g: Lateral acceleration in g
            roll_deg: Body roll angle in degrees
            steady_state_threshold: Roll rate threshold for steady-state (deg/s)

        Returns:
            Tuple of (gradient_deg_per_g, r_squared)
        """
        # Filter for steady-state conditions
        roll_rate = np.gradient(roll_deg, 1/self.sample_rate)
        steady_mask = np.abs(roll_rate) < steady_state_threshold

        # Also require some lateral g
        active_mask = np.abs(lateral_g) > 0.1

        mask = steady_mask & active_mask

        if np.sum(mask) < 50:
            return 0.0, 0.0

        lat_g = lateral_g[mask]
        roll = roll_deg[mask]

        # Linear regression through origin (roll = gradient * lateral_g)
        # Standard linear regression for better fit
        slope, intercept, r_value, _, _ = stats.linregress(lat_g, roll)

        return abs(slope), r_value ** 2

    def calculate_pitch_gradient(
        self,
        longitudinal_g: np.ndarray,
        pitch_deg: np.ndarray
    ) -> Tuple[float, float]:
        """
        Calculate pitch gradient (degrees per g longitudinal acceleration).

        Args:
            longitudinal_g: Longitudinal acceleration in g
            pitch_deg: Body pitch angle in degrees

        Returns:
            Tuple of (gradient_deg_per_g, r_squared)
        """
        # Filter for steady-state
        pitch_rate = np.gradient(pitch_deg, 1/self.sample_rate)
        steady_mask = np.abs(pitch_rate) < 5.0

        # Require some longitudinal g
        active_mask = np.abs(longitudinal_g) > 0.1

        mask = steady_mask & active_mask

        if np.sum(mask) < 50:
            return 0.0, 0.0

        lon_g = longitudinal_g[mask]
        pitch = pitch_deg[mask]

        slope, intercept, r_value, _, _ = stats.linregress(lon_g, pitch)

        return abs(slope), r_value ** 2

    def calculate_roll_distribution(
        self,
        front_roll_deg: np.ndarray,
        rear_roll_deg: np.ndarray
    ) -> Tuple[float, float]:
        """
        Calculate roll couple distribution (front vs rear).

        Args:
            front_roll_deg: Front axle roll angle
            rear_roll_deg: Rear axle roll angle

        Returns:
            Tuple of (front_pct, rear_pct)
        """
        # In steady-state cornering, total roll = front + rear contribution
        # The relative stiffness determines distribution

        front_avg = np.mean(np.abs(front_roll_deg))
        rear_avg = np.mean(np.abs(rear_roll_deg))

        total = front_avg + rear_avg
        if total > 0:
            # More roll at an end means less stiffness there
            # So invert the ratio for stiffness
            front_pct = (1 - front_avg / total) * 100
            rear_pct = (1 - rear_avg / total) * 100
        else:
            front_pct = 50.0
            rear_pct = 50.0

        return front_pct, rear_pct

    def estimate_understeer_gradient(
        self,
        steering_angle_deg: np.ndarray,
        lateral_g: np.ndarray,
        speed_mph: np.ndarray,
        wheelbase_m: Optional[float] = None
    ) -> Tuple[float, str]:
        """
        Estimate understeer gradient from steering data.

        Understeer gradient K = d(steering_angle) / d(lateral_g) - L/R
        where L is wheelbase and R is turn radius.

        Args:
            steering_angle_deg: Front wheel steering angle
            lateral_g: Lateral acceleration
            speed_mph: Vehicle speed
            wheelbase_m: Wheelbase (uses instance value if None)

        Returns:
            Tuple of (understeer_gradient, balance_rating)
        """
        if wheelbase_m is None:
            wheelbase_m = self.wheelbase

        # Filter for cornering conditions
        mask = (np.abs(lateral_g) > 0.3) & (speed_mph > 20)

        if np.sum(mask) < 50:
            return 0.0, "unknown"

        steer = np.abs(steering_angle_deg[mask])
        lat_g = np.abs(lateral_g[mask])

        # Calculate Ackermann steering angle for comparison
        speed_m_s = speed_mph[mask] * 0.44704
        radius = speed_m_s ** 2 / (lat_g * 9.81 + 0.001)
        ackermann = np.degrees(wheelbase_m / radius)

        # Understeer = actual steering - ackermann
        understeer = steer - ackermann

        # Fit gradient
        slope, _, _, _, _ = stats.linregress(lat_g, understeer)

        # Classify balance
        if slope > 2.0:
            balance = "understeer"
        elif slope < -2.0:
            balance = "oversteer"
        else:
            balance = "neutral"

        return slope, balance

    def analyze_transient_response(
        self,
        lateral_g: np.ndarray,
        roll_deg: np.ndarray,
        yaw_rate_dps: np.ndarray,
        step_threshold_g: float = 0.3
    ) -> List[TransientEvent]:
        """
        Analyze transient response to steering inputs.

        Detects step inputs in lateral g and measures roll response.

        Args:
            lateral_g: Lateral acceleration
            roll_deg: Body roll angle
            yaw_rate_dps: Yaw rate
            step_threshold_g: Minimum g-step to analyze

        Returns:
            List of TransientEvent objects
        """
        events = []

        # Find step changes in lateral g
        lat_g_smooth = self.lpf.filter(lateral_g)
        lat_g_rate = np.gradient(lat_g_smooth, 1/self.sample_rate)

        # Detect step inputs (high rate of change followed by stabilization)
        step_candidates = np.where(np.abs(lat_g_rate) > 2.0)[0]  # > 2g/s

        if len(step_candidates) == 0:
            return events

        # Group into events
        event_starts = [step_candidates[0]]
        for i in range(1, len(step_candidates)):
            if step_candidates[i] - step_candidates[i-1] > int(0.5 * self.sample_rate):
                event_starts.append(step_candidates[i])

        # Analyze each event
        for start_idx in event_starts:
            # Find window (2 seconds after step)
            end_idx = min(start_idx + int(2.0 * self.sample_rate), len(lateral_g))

            if end_idx - start_idx < int(0.5 * self.sample_rate):
                continue

            window_lat_g = lateral_g[start_idx:end_idx]
            window_roll = roll_deg[start_idx:end_idx]

            # Check if this is a real step
            g_before = lateral_g[max(0, start_idx - 10):start_idx].mean() if start_idx > 10 else 0
            g_after = window_lat_g[-int(0.2*self.sample_rate):].mean()
            g_step = abs(g_after - g_before)

            if g_step < step_threshold_g:
                continue

            # Measure roll response
            roll_before = roll_deg[max(0, start_idx - 10):start_idx].mean() if start_idx > 10 else 0
            steady_state_roll = window_roll[-int(0.2*self.sample_rate):].mean()
            roll_change = steady_state_roll - roll_before

            # Find time to reach 63% of final value (time constant)
            target_63 = roll_before + 0.632 * roll_change
            time_constant = 0.0

            for i, roll in enumerate(window_roll):
                if abs(roll - roll_before) >= abs(target_63 - roll_before):
                    time_constant = i / self.sample_rate
                    break

            # Find peak roll (overshoot)
            if roll_change > 0:
                peak_roll = np.max(window_roll)
            else:
                peak_roll = np.min(window_roll)

            overshoot_pct = 0.0
            if abs(roll_change) > 0.1:
                overshoot_pct = (abs(peak_roll - roll_before) - abs(roll_change)) / abs(roll_change) * 100
                overshoot_pct = max(0, overshoot_pct)

            # Find settling time (within 5% of steady state)
            settling_time = 2.0  # Default to full window
            settle_threshold = abs(roll_change) * 0.05

            for i in range(len(window_roll) - 1, 0, -1):
                if abs(window_roll[i] - steady_state_roll) > settle_threshold:
                    settling_time = i / self.sample_rate
                    break

            events.append(TransientEvent(
                start_time=start_idx / self.sample_rate,
                end_time=end_idx / self.sample_rate,
                lateral_g_step=g_step,
                roll_response_time=time_constant,
                roll_overshoot=overshoot_pct,
                settling_time=settling_time,
                steady_state_roll=abs(roll_change)
            ))

        return events

    def analyze(
        self,
        pot_fl: np.ndarray,
        pot_fr: np.ndarray,
        pot_rl: np.ndarray,
        pot_rr: np.ndarray,
        lateral_g: np.ndarray,
        longitudinal_g: np.ndarray,
        yaw_rate_dps: np.ndarray,
        steering_angle_deg: Optional[np.ndarray] = None,
        speed_mph: Optional[np.ndarray] = None
    ) -> HandlingMetrics:
        """
        Perform complete handling analysis.

        Args:
            pot_*: Potentiometer data for each corner
            lateral_g: Lateral acceleration
            longitudinal_g: Longitudinal acceleration
            yaw_rate_dps: Yaw rate
            steering_angle_deg: Optional steering angle
            speed_mph: Optional vehicle speed

        Returns:
            HandlingMetrics with all analysis results
        """
        metrics = HandlingMetrics()

        # Calculate body angles
        front_roll, rear_roll = self.calculate_body_roll(
            pot_fl, pot_fr, pot_rl, pot_rr
        )
        total_roll = (front_roll + rear_roll) / 2
        pitch = self.calculate_body_pitch(pot_fl, pot_fr, pot_rl, pot_rr)

        # Roll gradient
        gradient, r2 = self.calculate_roll_gradient(lateral_g, total_roll)
        metrics.roll_gradient_deg_per_g = gradient
        metrics.roll_gradient_r_squared = r2
        metrics.max_roll_deg = float(np.max(np.abs(total_roll)))

        # Roll distribution
        front_pct, rear_pct = self.calculate_roll_distribution(front_roll, rear_roll)
        metrics.roll_stiffness_front_pct = front_pct
        metrics.roll_stiffness_rear_pct = rear_pct

        # Pitch gradient
        pitch_grad, pitch_r2 = self.calculate_pitch_gradient(longitudinal_g, pitch)
        metrics.pitch_gradient_deg_per_g = pitch_grad
        metrics.pitch_gradient_r_squared = pitch_r2
        metrics.max_pitch_deg = float(np.max(np.abs(pitch)))

        # Anti-dive/squat estimates (simplified)
        braking_mask = longitudinal_g < -0.2
        accel_mask = longitudinal_g > 0.2

        if np.sum(braking_mask) > 50:
            expected_pitch = np.mean(longitudinal_g[braking_mask]) * 5  # Rough estimate
            actual_pitch = np.mean(pitch[braking_mask])
            if expected_pitch != 0:
                metrics.anti_dive_pct = (1 - actual_pitch / expected_pitch) * 100

        # Understeer gradient
        if steering_angle_deg is not None and speed_mph is not None:
            us_grad, balance = self.estimate_understeer_gradient(
                steering_angle_deg, lateral_g, speed_mph
            )
            metrics.understeer_gradient_deg_per_g = us_grad
            metrics.balance_rating = balance

        # Transient response
        events = self.analyze_transient_response(lateral_g, total_roll, yaw_rate_dps)

        if events:
            metrics.roll_time_constant_sec = np.mean([e.roll_response_time for e in events])
            metrics.roll_settling_time_sec = np.mean([e.settling_time for e in events])
            metrics.roll_overshoot_pct = np.mean([e.roll_overshoot for e in events])

        return metrics

    def generate_gg_diagram_data(
        self,
        lateral_g: np.ndarray,
        longitudinal_g: np.ndarray,
        bin_size: float = 0.1
    ) -> Dict[str, np.ndarray]:
        """
        Generate data for G-G diagram (friction circle).

        Args:
            lateral_g: Lateral acceleration
            longitudinal_g: Longitudinal acceleration
            bin_size: Bin size for histogram

        Returns:
            Dictionary with g-g diagram data
        """
        # Create 2D histogram
        bins = np.arange(-1.5, 1.5 + bin_size, bin_size)

        hist, xedges, yedges = np.histogram2d(
            lateral_g, longitudinal_g, bins=bins
        )

        # Find convex hull / boundary
        combined_g = np.sqrt(lateral_g**2 + longitudinal_g**2)

        return {
            'lateral_g': lateral_g,
            'longitudinal_g': longitudinal_g,
            'histogram': hist,
            'bin_edges': bins,
            'max_combined_g': float(np.max(combined_g)),
            'max_lateral_g': float(np.max(np.abs(lateral_g))),
            'max_braking_g': float(np.min(longitudinal_g)),
            'max_accel_g': float(np.max(longitudinal_g))
        }
