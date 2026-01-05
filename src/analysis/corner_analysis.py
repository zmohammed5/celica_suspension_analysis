"""
Corner Weight and Load Transfer Analysis Module.

This module provides analysis of:
- Static corner weights
- Dynamic weight transfer
- Cross-weight percentage
- Corner compliance
- Tire load variation
"""

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import stats

logger = logging.getLogger(__name__)


@dataclass
class CornerWeights:
    """Static corner weight distribution."""
    front_left_lbs: float = 0.0
    front_right_lbs: float = 0.0
    rear_left_lbs: float = 0.0
    rear_right_lbs: float = 0.0

    @property
    def total_weight_lbs(self) -> float:
        """Total vehicle weight."""
        return (self.front_left_lbs + self.front_right_lbs +
                self.rear_left_lbs + self.rear_right_lbs)

    @property
    def front_weight_pct(self) -> float:
        """Front weight percentage."""
        total = self.total_weight_lbs
        if total > 0:
            return (self.front_left_lbs + self.front_right_lbs) / total * 100
        return 50.0

    @property
    def left_weight_pct(self) -> float:
        """Left side weight percentage."""
        total = self.total_weight_lbs
        if total > 0:
            return (self.front_left_lbs + self.rear_left_lbs) / total * 100
        return 50.0

    @property
    def cross_weight_pct(self) -> float:
        """Cross weight (wedge) percentage."""
        total = self.total_weight_lbs
        if total > 0:
            return (self.front_left_lbs + self.rear_right_lbs) / total * 100
        return 50.0

    def as_dict(self) -> Dict[str, float]:
        """Convert to dictionary."""
        return {
            'front_left_lbs': self.front_left_lbs,
            'front_right_lbs': self.front_right_lbs,
            'rear_left_lbs': self.rear_left_lbs,
            'rear_right_lbs': self.rear_right_lbs,
            'total_lbs': self.total_weight_lbs,
            'front_pct': self.front_weight_pct,
            'left_pct': self.left_weight_pct,
            'cross_weight_pct': self.cross_weight_pct
        }


@dataclass
class DynamicLoadTransfer:
    """Dynamic load transfer calculations."""

    # Lateral load transfer
    lateral_transfer_front_lbs: float = 0.0
    lateral_transfer_rear_lbs: float = 0.0
    lateral_transfer_total_lbs: float = 0.0

    # Longitudinal load transfer
    longitudinal_transfer_lbs: float = 0.0

    # Load transfer distribution
    lateral_transfer_front_pct: float = 50.0  # % of lateral transfer at front

    # Peak loads
    max_inside_front_load_lbs: float = 0.0
    max_outside_front_load_lbs: float = 0.0
    max_inside_rear_load_lbs: float = 0.0
    max_outside_rear_load_lbs: float = 0.0


@dataclass
class TireLoadVariation:
    """Tire load variation statistics."""
    corner: str
    mean_load_lbs: float = 0.0
    std_load_lbs: float = 0.0
    min_load_lbs: float = 0.0
    max_load_lbs: float = 0.0
    coefficient_of_variation: float = 0.0
    time_unloaded_pct: float = 0.0  # % time with < 10% of mean load


class CornerAnalyzer:
    """
    Corner Weight and Load Transfer Analyzer.

    Analyzes static and dynamic weight distribution using
    suspension travel and acceleration data.
    """

    CORNERS = ['front_left', 'front_right', 'rear_left', 'rear_right']

    def __init__(
        self,
        total_weight_lbs: float = 2500.0,
        wheelbase_in: float = 102.4,
        track_front_in: float = 59.8,
        track_rear_in: float = 59.4,
        cg_height_in: float = 18.5,
        spring_rates_lbs_in: Optional[Dict[str, float]] = None
    ):
        """
        Initialize corner analyzer.

        Args:
            total_weight_lbs: Total vehicle weight
            wheelbase_in: Wheelbase in inches
            track_front_in: Front track width in inches
            track_rear_in: Rear track width in inches
            cg_height_in: CG height in inches
            spring_rates_lbs_in: Spring rates per corner (lbs/in)
        """
        self.total_weight = total_weight_lbs
        self.wheelbase = wheelbase_in
        self.track_front = track_front_in
        self.track_rear = track_rear_in
        self.cg_height = cg_height_in

        self.spring_rates = spring_rates_lbs_in or {
            'front_left': 450, 'front_right': 450,
            'rear_left': 350, 'rear_right': 350
        }

    def estimate_static_weights_from_travel(
        self,
        pot_fl_mm: np.ndarray,
        pot_fr_mm: np.ndarray,
        pot_rl_mm: np.ndarray,
        pot_rr_mm: np.ndarray,
        settle_time_sec: float = 5.0,
        sample_rate_hz: float = 100.0
    ) -> CornerWeights:
        """
        Estimate static corner weights from potentiometer readings.

        Uses spring rates and relative compression to estimate
        corner weight distribution.

        Args:
            pot_*: Potentiometer data for each corner (mm)
            settle_time_sec: Time at start to use for static measurement
            sample_rate_hz: Data sample rate

        Returns:
            CornerWeights with estimated static weights
        """
        # Use first N samples (static/settled)
        n_samples = int(settle_time_sec * sample_rate_hz)

        fl_travel = np.mean(pot_fl_mm[:n_samples]) if len(pot_fl_mm) >= n_samples else np.mean(pot_fl_mm)
        fr_travel = np.mean(pot_fr_mm[:n_samples]) if len(pot_fr_mm) >= n_samples else np.mean(pot_fr_mm)
        rl_travel = np.mean(pot_rl_mm[:n_samples]) if len(pot_rl_mm) >= n_samples else np.mean(pot_rl_mm)
        rr_travel = np.mean(pot_rr_mm[:n_samples]) if len(pot_rr_mm) >= n_samples else np.mean(pot_rr_mm)

        # Convert mm to inches
        travels_in = {
            'front_left': fl_travel / 25.4,
            'front_right': fr_travel / 25.4,
            'rear_left': rl_travel / 25.4,
            'rear_right': rr_travel / 25.4
        }

        # Calculate relative loads from spring compression
        # More compression = more load
        relative_loads = {}
        for corner in self.CORNERS:
            # Assuming potentiometer reads positive for compression
            relative_loads[corner] = travels_in[corner] * self.spring_rates[corner]

        # Normalize to total weight
        total_relative = sum(relative_loads.values())

        if total_relative > 0:
            scale_factor = self.total_weight / total_relative
        else:
            scale_factor = self.total_weight / 4

        return CornerWeights(
            front_left_lbs=relative_loads['front_left'] * scale_factor,
            front_right_lbs=relative_loads['front_right'] * scale_factor,
            rear_left_lbs=relative_loads['rear_left'] * scale_factor,
            rear_right_lbs=relative_loads['rear_right'] * scale_factor
        )

    def calculate_lateral_load_transfer(
        self,
        lateral_g: float,
        include_unsprung: bool = True
    ) -> Tuple[float, float]:
        """
        Calculate lateral load transfer.

        Args:
            lateral_g: Lateral acceleration in g
            include_unsprung: Include unsprung mass transfer

        Returns:
            Tuple of (front_transfer_lbs, rear_transfer_lbs)
        """
        # Load transfer = (Weight * ay * h_cg) / track
        weight_front = self.total_weight * 0.6  # Assume 60/40 distribution
        weight_rear = self.total_weight * 0.4

        transfer_front = (weight_front * lateral_g * self.cg_height) / self.track_front
        transfer_rear = (weight_rear * lateral_g * self.cg_height) / self.track_rear

        return transfer_front, transfer_rear

    def calculate_longitudinal_load_transfer(
        self,
        longitudinal_g: float
    ) -> float:
        """
        Calculate longitudinal load transfer.

        Args:
            longitudinal_g: Longitudinal acceleration in g

        Returns:
            Load transfer in lbs (positive = forward transfer)
        """
        # Load transfer = (Weight * ax * h_cg) / wheelbase
        return (self.total_weight * longitudinal_g * self.cg_height) / self.wheelbase

    def calculate_dynamic_loads(
        self,
        static_weights: CornerWeights,
        lateral_g: np.ndarray,
        longitudinal_g: np.ndarray
    ) -> Dict[str, np.ndarray]:
        """
        Calculate dynamic corner loads over time.

        Args:
            static_weights: Static corner weights
            lateral_g: Lateral acceleration array
            longitudinal_g: Longitudinal acceleration array

        Returns:
            Dictionary of load arrays per corner
        """
        n = len(lateral_g)
        loads = {corner: np.zeros(n) for corner in self.CORNERS}

        # Start with static weights
        loads['front_left'][:] = static_weights.front_left_lbs
        loads['front_right'][:] = static_weights.front_right_lbs
        loads['rear_left'][:] = static_weights.rear_left_lbs
        loads['rear_right'][:] = static_weights.rear_right_lbs

        for i in range(n):
            lat_g = lateral_g[i]
            lon_g = longitudinal_g[i]

            # Lateral transfer
            front_lat_transfer = (self.total_weight * 0.6 * lat_g * self.cg_height) / self.track_front
            rear_lat_transfer = (self.total_weight * 0.4 * lat_g * self.cg_height) / self.track_rear

            # Longitudinal transfer
            lon_transfer = (self.total_weight * lon_g * self.cg_height) / self.wheelbase

            # Apply transfers (positive lat_g = right turn = load to left)
            loads['front_left'][i] += front_lat_transfer + lon_transfer / 2
            loads['front_right'][i] -= front_lat_transfer + lon_transfer / 2
            loads['rear_left'][i] += rear_lat_transfer - lon_transfer / 2
            loads['rear_right'][i] -= rear_lat_transfer - lon_transfer / 2

            # Clamp to non-negative (can't have negative tire load)
            for corner in self.CORNERS:
                loads[corner][i] = max(0, loads[corner][i])

        return loads

    def analyze_load_variation(
        self,
        loads: np.ndarray,
        corner: str
    ) -> TireLoadVariation:
        """
        Analyze tire load variation statistics.

        Args:
            loads: Load array for the corner
            corner: Corner identifier

        Returns:
            TireLoadVariation with statistics
        """
        mean_load = float(np.mean(loads))
        std_load = float(np.std(loads))

        # Time unloaded (< 10% of mean)
        unload_threshold = mean_load * 0.1
        time_unloaded = np.sum(loads < unload_threshold) / len(loads) * 100

        cv = (std_load / mean_load * 100) if mean_load > 0 else 0

        return TireLoadVariation(
            corner=corner,
            mean_load_lbs=mean_load,
            std_load_lbs=std_load,
            min_load_lbs=float(np.min(loads)),
            max_load_lbs=float(np.max(loads)),
            coefficient_of_variation=cv,
            time_unloaded_pct=time_unloaded
        )

    def calculate_compliance(
        self,
        pot_data: np.ndarray,
        accel_data: np.ndarray,
        corner: str
    ) -> float:
        """
        Estimate corner compliance (mm deflection per g).

        Higher compliance = softer suspension.

        Args:
            pot_data: Potentiometer data (mm)
            accel_data: Vertical acceleration (g)
            corner: Corner identifier

        Returns:
            Compliance in mm/g
        """
        # Filter for high-g events
        high_g_mask = np.abs(accel_data) > 0.5

        if np.sum(high_g_mask) < 50:
            return 0.0

        # Linear regression of displacement vs acceleration
        slope, _, r_value, _, _ = stats.linregress(
            accel_data[high_g_mask],
            pot_data[high_g_mask]
        )

        return abs(slope)

    def analyze(
        self,
        pot_fl: np.ndarray,
        pot_fr: np.ndarray,
        pot_rl: np.ndarray,
        pot_rr: np.ndarray,
        lateral_g: np.ndarray,
        longitudinal_g: np.ndarray,
        accel_fl_z: Optional[np.ndarray] = None,
        accel_fr_z: Optional[np.ndarray] = None,
        accel_rl_z: Optional[np.ndarray] = None,
        accel_rr_z: Optional[np.ndarray] = None,
        sample_rate_hz: float = 100.0
    ) -> Dict:
        """
        Perform complete corner analysis.

        Args:
            pot_*: Potentiometer data for each corner
            lateral_g: Lateral acceleration
            longitudinal_g: Longitudinal acceleration
            accel_*_z: Optional vertical acceleration per corner
            sample_rate_hz: Sample rate

        Returns:
            Dictionary with all analysis results
        """
        results = {}

        # Static weights
        static_weights = self.estimate_static_weights_from_travel(
            pot_fl, pot_fr, pot_rl, pot_rr,
            sample_rate_hz=sample_rate_hz
        )
        results['static_weights'] = static_weights.as_dict()

        # Dynamic loads
        dynamic_loads = self.calculate_dynamic_loads(
            static_weights, lateral_g, longitudinal_g
        )

        # Load variation per corner
        results['load_variation'] = {}
        for corner in self.CORNERS:
            variation = self.analyze_load_variation(dynamic_loads[corner], corner)
            results['load_variation'][corner] = {
                'mean_lbs': variation.mean_load_lbs,
                'std_lbs': variation.std_load_lbs,
                'min_lbs': variation.min_load_lbs,
                'max_lbs': variation.max_load_lbs,
                'cv_pct': variation.coefficient_of_variation,
                'time_unloaded_pct': variation.time_unloaded_pct
            }

        # Peak load transfer
        results['peak_lateral_transfer'] = {
            'max_g': float(np.max(np.abs(lateral_g))),
            'front_lbs': float(np.max(np.abs(dynamic_loads['front_left'] - static_weights.front_left_lbs))),
            'rear_lbs': float(np.max(np.abs(dynamic_loads['rear_left'] - static_weights.rear_left_lbs)))
        }

        results['peak_longitudinal_transfer'] = {
            'max_braking_g': float(np.min(longitudinal_g)),
            'max_accel_g': float(np.max(longitudinal_g)),
            'transfer_lbs': float(np.max(np.abs(
                (dynamic_loads['front_left'] + dynamic_loads['front_right']) -
                (static_weights.front_left_lbs + static_weights.front_right_lbs)
            )))
        }

        # Compliance
        if all(x is not None for x in [accel_fl_z, accel_fr_z, accel_rl_z, accel_rr_z]):
            results['compliance'] = {
                'front_left': self.calculate_compliance(pot_fl, accel_fl_z, 'front_left'),
                'front_right': self.calculate_compliance(pot_fr, accel_fr_z, 'front_right'),
                'rear_left': self.calculate_compliance(pot_rl, accel_rl_z, 'rear_left'),
                'rear_right': self.calculate_compliance(pot_rr, accel_rr_z, 'rear_right')
            }

        # Dynamic loads for plotting
        results['dynamic_loads'] = dynamic_loads

        return results

    def calculate_ideal_cross_weight(
        self,
        current_weights: CornerWeights,
        target_cross_pct: float = 50.0
    ) -> Dict[str, float]:
        """
        Calculate adjustments needed for target cross weight.

        Args:
            current_weights: Current corner weights
            target_cross_pct: Target cross weight percentage

        Returns:
            Dictionary of suggested adjustments per corner
        """
        current_cross = current_weights.cross_weight_pct
        error = target_cross_pct - current_cross

        # To increase cross weight: raise FL/RR or lower FR/RL
        adjustment_lbs = error * current_weights.total_weight_lbs / 100 / 2

        return {
            'front_left': adjustment_lbs,
            'front_right': -adjustment_lbs,
            'rear_left': -adjustment_lbs,
            'rear_right': adjustment_lbs,
            'current_cross_pct': current_cross,
            'target_cross_pct': target_cross_pct,
            'error_pct': error
        }
