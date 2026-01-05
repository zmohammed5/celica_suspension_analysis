"""
Damper/Shock Absorber Analysis Module.

This module provides comprehensive analysis of damper performance including:
- Force-velocity curve estimation
- Velocity histogram analysis
- Compression vs rebound characterization
- Temperature fade analysis
- Damper dyno curve generation
"""

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import stats

from ..utils.calculations import calculate_derivative, calculate_force_from_acceleration
from ..utils.filters import LowPassFilter, SavitzkyGolayFilter

logger = logging.getLogger(__name__)


@dataclass
class DamperCharacteristics:
    """Characteristics of a single damper."""
    corner: str

    # Force-velocity data
    velocities: np.ndarray = field(default_factory=lambda: np.array([]))
    forces: np.ndarray = field(default_factory=lambda: np.array([]))

    # Compression (positive velocity = compression)
    compression_coefficient: float = 0.0  # N/(mm/s)
    compression_force_at_50: float = 0.0  # Force at 50mm/s
    compression_force_at_100: float = 0.0
    compression_force_at_200: float = 0.0

    # Rebound (negative velocity)
    rebound_coefficient: float = 0.0
    rebound_force_at_50: float = 0.0
    rebound_force_at_100: float = 0.0
    rebound_force_at_200: float = 0.0

    # Ratio
    compression_rebound_ratio: float = 0.0

    # Velocity statistics
    mean_velocity: float = 0.0
    max_compression_velocity: float = 0.0
    max_rebound_velocity: float = 0.0
    rms_velocity: float = 0.0

    # Temperature correlation
    temp_fade_coefficient: float = 0.0  # % force reduction per degree F


@dataclass
class VelocityHistogram:
    """Velocity distribution histogram."""
    bins: np.ndarray
    counts: np.ndarray
    percentage: np.ndarray
    total_samples: int


class DamperAnalyzer:
    """
    Damper Performance Analyzer.

    Analyzes suspension damper behavior using potentiometer displacement
    and accelerometer data to characterize force-velocity relationships.
    """

    CORNERS = ['front_left', 'front_right', 'rear_left', 'rear_right']

    # Default velocity bins for histograms (mm/s)
    DEFAULT_VELOCITY_BINS = np.array([
        -500, -400, -300, -200, -150, -100, -75, -50, -25, 0,
        25, 50, 75, 100, 150, 200, 300, 400, 500
    ])

    def __init__(
        self,
        sample_rate_hz: float = 100.0,
        unsprung_masses_kg: Optional[Dict[str, float]] = None,
        motion_ratios: Optional[Dict[str, float]] = None,
        velocity_filter_hz: float = 20.0
    ):
        """
        Initialize damper analyzer.

        Args:
            sample_rate_hz: Data sample rate
            unsprung_masses_kg: Unsprung mass per corner for force calculation
            motion_ratios: Motion ratio per corner (shock travel / wheel travel)
            velocity_filter_hz: Low-pass filter cutoff for velocity calculation
        """
        self.sample_rate = sample_rate_hz
        self.unsprung_masses = unsprung_masses_kg or {
            'front_left': 38.5, 'front_right': 38.5,
            'rear_left': 34.0, 'rear_right': 34.0
        }
        self.motion_ratios = motion_ratios or {
            'front_left': 0.95, 'front_right': 0.95,
            'rear_left': 0.90, 'rear_right': 0.90
        }

        self.velocity_filter = SavitzkyGolayFilter(window_size=11, polynomial_order=3)
        self.lpf = LowPassFilter(velocity_filter_hz, sample_rate_hz)

    def calculate_damper_velocity(
        self,
        position_mm: np.ndarray,
        filter_velocity: bool = True
    ) -> np.ndarray:
        """
        Calculate damper velocity from position data.

        Args:
            position_mm: Damper position in mm
            filter_velocity: Whether to apply smoothing filter

        Returns:
            Velocity in mm/s
        """
        velocity = calculate_derivative(
            position_mm,
            self.sample_rate,
            method='savgol',
            smooth=filter_velocity
        )

        if filter_velocity:
            velocity = self.lpf.filter(velocity)

        return velocity

    def estimate_damper_force(
        self,
        corner: str,
        accel_z_g: np.ndarray,
        velocity_mm_s: np.ndarray,
        include_friction: bool = True,
        friction_force_n: float = 50.0
    ) -> np.ndarray:
        """
        Estimate damper force from unsprung mass acceleration.

        F_damper = m * a - F_spring - F_friction

        For this simplified analysis, we focus on dynamic force:
        F_damper ≈ m_unsprung * a_z

        Args:
            corner: Corner identifier
            accel_z_g: Vertical acceleration in g
            velocity_mm_s: Damper velocity
            include_friction: Include friction compensation
            friction_force_n: Static friction force estimate

        Returns:
            Estimated damper force in Newtons
        """
        mass = self.unsprung_masses.get(corner, 35.0)

        # F = m * a
        force = calculate_force_from_acceleration(accel_z_g, mass)

        # Friction compensation (Coulomb friction model)
        if include_friction:
            friction = np.sign(velocity_mm_s) * friction_force_n
            force = force - friction

        return force

    def analyze_velocity_histogram(
        self,
        velocity_mm_s: np.ndarray,
        bins: Optional[np.ndarray] = None
    ) -> VelocityHistogram:
        """
        Create velocity distribution histogram.

        Shows time spent at each damper velocity, useful for
        understanding operating conditions.

        Args:
            velocity_mm_s: Velocity data
            bins: Histogram bin edges

        Returns:
            VelocityHistogram with distribution data
        """
        if bins is None:
            bins = self.DEFAULT_VELOCITY_BINS

        counts, bin_edges = np.histogram(velocity_mm_s, bins=bins)
        total = len(velocity_mm_s)
        percentage = (counts / total) * 100 if total > 0 else counts

        return VelocityHistogram(
            bins=bins,
            counts=counts,
            percentage=percentage,
            total_samples=total
        )

    def calculate_force_velocity_curve(
        self,
        velocity_mm_s: np.ndarray,
        force_n: np.ndarray,
        velocity_bins: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Calculate average force at each velocity bin.

        Args:
            velocity_mm_s: Damper velocity
            force_n: Estimated damper force
            velocity_bins: Bin edges for velocity grouping

        Returns:
            Tuple of (bin_centers, mean_forces, std_forces)
        """
        if velocity_bins is None:
            velocity_bins = self.DEFAULT_VELOCITY_BINS

        bin_centers = (velocity_bins[:-1] + velocity_bins[1:]) / 2
        mean_forces = np.zeros(len(bin_centers))
        std_forces = np.zeros(len(bin_centers))

        for i in range(len(bin_centers)):
            mask = (velocity_mm_s >= velocity_bins[i]) & (velocity_mm_s < velocity_bins[i+1])
            if np.sum(mask) > 0:
                mean_forces[i] = np.mean(force_n[mask])
                std_forces[i] = np.std(force_n[mask])

        return bin_centers, mean_forces, std_forces

    def fit_damping_coefficient(
        self,
        velocity_mm_s: np.ndarray,
        force_n: np.ndarray,
        velocity_range: Tuple[float, float] = (10, 200)
    ) -> Tuple[float, float, float]:
        """
        Fit linear damping coefficient to force-velocity data.

        F = c * v (linear damping model)

        Args:
            velocity_mm_s: Velocity data
            force_n: Force data
            velocity_range: Velocity range to fit (absolute values)

        Returns:
            Tuple of (coefficient, intercept, r_squared)
        """
        # Filter to velocity range
        abs_vel = np.abs(velocity_mm_s)
        mask = (abs_vel >= velocity_range[0]) & (abs_vel <= velocity_range[1])

        if np.sum(mask) < 10:
            return 0.0, 0.0, 0.0

        vel = velocity_mm_s[mask]
        force = force_n[mask]

        # Linear regression
        slope, intercept, r_value, _, _ = stats.linregress(vel, force)

        return slope, intercept, r_value ** 2

    def analyze_corner(
        self,
        corner: str,
        position_mm: np.ndarray,
        accel_z_g: np.ndarray,
        temperature_f: Optional[np.ndarray] = None
    ) -> DamperCharacteristics:
        """
        Perform complete analysis for a single corner.

        Args:
            corner: Corner identifier
            position_mm: Damper position data
            accel_z_g: Unsprung mass vertical acceleration
            temperature_f: Optional shock body temperature

        Returns:
            DamperCharacteristics with all analysis results
        """
        # Calculate velocity
        velocity = self.calculate_damper_velocity(position_mm)

        # Estimate force
        force = self.estimate_damper_force(corner, accel_z_g, velocity)

        # Create characteristics
        chars = DamperCharacteristics(
            corner=corner,
            velocities=velocity,
            forces=force
        )

        # Velocity statistics
        chars.mean_velocity = float(np.mean(np.abs(velocity)))
        chars.max_compression_velocity = float(np.max(velocity))
        chars.max_rebound_velocity = float(np.min(velocity))
        chars.rms_velocity = float(np.sqrt(np.mean(velocity ** 2)))

        # Fit compression coefficient (positive velocity)
        comp_mask = velocity > 10
        if np.sum(comp_mask) > 10:
            coef, _, _ = self.fit_damping_coefficient(
                velocity[comp_mask], force[comp_mask], (10, 200)
            )
            chars.compression_coefficient = coef

        # Fit rebound coefficient (negative velocity)
        reb_mask = velocity < -10
        if np.sum(reb_mask) > 10:
            coef, _, _ = self.fit_damping_coefficient(
                velocity[reb_mask], force[reb_mask], (10, 200)
            )
            chars.rebound_coefficient = abs(coef)

        # Calculate forces at specific velocities
        vel_centers, mean_forces, _ = self.calculate_force_velocity_curve(velocity, force)

        for target_vel in [50, 100, 200]:
            # Compression
            idx = np.argmin(np.abs(vel_centers - target_vel))
            setattr(chars, f'compression_force_at_{target_vel}', abs(mean_forces[idx]))

            # Rebound
            idx = np.argmin(np.abs(vel_centers + target_vel))
            setattr(chars, f'rebound_force_at_{target_vel}', abs(mean_forces[idx]))

        # Compression/rebound ratio
        if chars.rebound_coefficient > 0:
            chars.compression_rebound_ratio = (
                chars.compression_coefficient / chars.rebound_coefficient
            )

        # Temperature fade analysis
        if temperature_f is not None and len(temperature_f) == len(force):
            chars.temp_fade_coefficient = self._analyze_temperature_fade(
                temperature_f, force, velocity
            )

        return chars

    def _analyze_temperature_fade(
        self,
        temperature_f: np.ndarray,
        force_n: np.ndarray,
        velocity_mm_s: np.ndarray
    ) -> float:
        """
        Analyze damper force reduction with temperature increase.

        Args:
            temperature_f: Temperature data
            force_n: Force data
            velocity_mm_s: Velocity data

        Returns:
            Fade coefficient (% force reduction per degree F)
        """
        # Focus on a specific velocity range for comparison
        vel_mask = (np.abs(velocity_mm_s) > 50) & (np.abs(velocity_mm_s) < 150)

        if np.sum(vel_mask) < 50:
            return 0.0

        temps = temperature_f[vel_mask]
        forces = np.abs(force_n[vel_mask])

        # Bin by temperature
        temp_min, temp_max = np.min(temps), np.max(temps)

        if temp_max - temp_min < 20:
            return 0.0  # Not enough temperature variation

        temp_bins = np.linspace(temp_min, temp_max, 5)
        mean_forces = []
        mean_temps = []

        for i in range(len(temp_bins) - 1):
            mask = (temps >= temp_bins[i]) & (temps < temp_bins[i+1])
            if np.sum(mask) > 10:
                mean_forces.append(np.mean(forces[mask]))
                mean_temps.append((temp_bins[i] + temp_bins[i+1]) / 2)

        if len(mean_forces) < 2:
            return 0.0

        # Linear regression
        slope, _, _, _, _ = stats.linregress(mean_temps, mean_forces)

        # Convert to percentage per degree
        base_force = mean_forces[0]
        if base_force > 0:
            fade_pct = (slope / base_force) * 100
        else:
            fade_pct = 0.0

        return fade_pct

    def analyze_all_corners(
        self,
        data: Dict[str, Dict[str, np.ndarray]]
    ) -> Dict[str, DamperCharacteristics]:
        """
        Analyze all four corners.

        Args:
            data: Dictionary with corner data containing:
                - 'position_mm': Potentiometer data
                - 'accel_z_g': Vertical acceleration
                - 'temperature_f': Optional temperature

        Returns:
            Dictionary of DamperCharacteristics per corner
        """
        results = {}

        for corner in self.CORNERS:
            if corner not in data:
                continue

            corner_data = data[corner]
            results[corner] = self.analyze_corner(
                corner=corner,
                position_mm=corner_data['position_mm'],
                accel_z_g=corner_data['accel_z_g'],
                temperature_f=corner_data.get('temperature_f')
            )

        return results

    def generate_dyno_curve(
        self,
        chars: DamperCharacteristics,
        velocity_range: Tuple[float, float] = (-400, 400),
        points: int = 50
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate idealized damper dyno curve from analysis.

        Args:
            chars: Damper characteristics from analysis
            velocity_range: Velocity range for curve
            points: Number of points

        Returns:
            Tuple of (velocities, forces)
        """
        velocities = np.linspace(velocity_range[0], velocity_range[1], points)
        forces = np.zeros_like(velocities)

        # Use linear model with measured coefficients
        for i, v in enumerate(velocities):
            if v >= 0:
                forces[i] = chars.compression_coefficient * v
            else:
                forces[i] = -chars.rebound_coefficient * v

        return velocities, forces

    def compare_corners(
        self,
        characteristics: Dict[str, DamperCharacteristics]
    ) -> Dict[str, float]:
        """
        Compare damper characteristics across corners.

        Args:
            characteristics: Analysis results for all corners

        Returns:
            Dictionary of comparison metrics
        """
        if len(characteristics) < 2:
            return {}

        results = {}

        # Front-rear comparison
        front_avg_comp = np.mean([
            characteristics.get('front_left', DamperCharacteristics('')).compression_coefficient,
            characteristics.get('front_right', DamperCharacteristics('')).compression_coefficient
        ])
        rear_avg_comp = np.mean([
            characteristics.get('rear_left', DamperCharacteristics('')).compression_coefficient,
            characteristics.get('rear_right', DamperCharacteristics('')).compression_coefficient
        ])

        if rear_avg_comp > 0:
            results['front_rear_comp_ratio'] = front_avg_comp / rear_avg_comp

        # Left-right balance
        left_avg = np.mean([
            characteristics.get('front_left', DamperCharacteristics('')).compression_force_at_100,
            characteristics.get('rear_left', DamperCharacteristics('')).compression_force_at_100
        ])
        right_avg = np.mean([
            characteristics.get('front_right', DamperCharacteristics('')).compression_force_at_100,
            characteristics.get('rear_right', DamperCharacteristics('')).compression_force_at_100
        ])

        total = left_avg + right_avg
        if total > 0:
            results['left_right_balance'] = left_avg / total * 100  # % on left

        # Coefficient of variation (consistency)
        comp_coeffs = [c.compression_coefficient for c in characteristics.values()]
        if np.mean(comp_coeffs) > 0:
            results['compression_cv'] = np.std(comp_coeffs) / np.mean(comp_coeffs) * 100

        return results
