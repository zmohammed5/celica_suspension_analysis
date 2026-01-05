"""
Session Comparison Analysis Module.

This module provides tools for comparing two data sessions,
useful for evaluating setup changes (before/after analysis).
"""

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy import stats

logger = logging.getLogger(__name__)


@dataclass
class ParameterComparison:
    """Comparison of a single parameter between sessions."""
    parameter_name: str
    session1_mean: float
    session2_mean: float
    session1_std: float
    session2_std: float
    absolute_change: float
    percent_change: float
    t_statistic: float
    p_value: float
    significant: bool
    improvement: Optional[bool] = None  # True if change is positive


@dataclass
class ComparisonResult:
    """Complete comparison between two sessions."""
    session1_id: str
    session2_id: str
    session1_name: str = ""
    session2_name: str = ""

    # Parameter comparisons
    parameters: Dict[str, ParameterComparison] = field(default_factory=dict)

    # Summary
    improvements: List[str] = field(default_factory=list)
    regressions: List[str] = field(default_factory=list)
    neutral: List[str] = field(default_factory=list)

    # Overall scores
    overall_improvement_pct: float = 0.0


class ComparisonAnalyzer:
    """
    Session Comparison Analyzer.

    Compares two data sessions to identify statistically significant
    differences, useful for evaluating the effect of setup changes.
    """

    # Parameters to compare and their interpretation
    # True = higher is better, False = lower is better, None = neutral
    PARAMETERS = {
        'damper_compression_coef': (False, "Compression damping"),
        'damper_rebound_coef': (False, "Rebound damping"),
        'natural_frequency': (None, "Natural frequency"),
        'damping_ratio': (None, "Damping ratio"),
        'roll_gradient': (False, "Roll gradient"),
        'pitch_gradient': (False, "Pitch gradient"),
        'max_roll': (False, "Maximum roll"),
        'understeer_gradient': (None, "Understeer gradient"),
        'roll_time_constant': (False, "Roll response time"),
        'cross_weight_deviation': (False, "Cross weight deviation"),
        'tire_load_variation': (False, "Tire load variation"),
        'harshness_count': (False, "Harshness events"),
        'weighted_rms_accel': (False, "Ride harshness"),
        'lap_time': (False, "Lap time"),
        'max_speed': (True, "Maximum speed"),
        'avg_speed': (True, "Average speed"),
        'max_lateral_g': (True, "Maximum lateral g"),
        'max_braking_g': (False, "Maximum braking g"),  # More negative is better
    }

    def __init__(
        self,
        significance_level: float = 0.05,
        min_samples: int = 100,
        significant_change_pct: float = 5.0
    ):
        """
        Initialize comparison analyzer.

        Args:
            significance_level: P-value threshold for significance
            min_samples: Minimum samples required for comparison
            significant_change_pct: Minimum % change to consider significant
        """
        self.significance_level = significance_level
        self.min_samples = min_samples
        self.significant_change_pct = significant_change_pct

    def compare_parameter(
        self,
        name: str,
        data1: np.ndarray,
        data2: np.ndarray,
        higher_is_better: Optional[bool] = None
    ) -> ParameterComparison:
        """
        Compare a single parameter between sessions.

        Args:
            name: Parameter name
            data1: Session 1 data
            data2: Session 2 data
            higher_is_better: Whether higher values are better

        Returns:
            ParameterComparison with statistical analysis
        """
        # Basic statistics
        mean1 = float(np.mean(data1))
        mean2 = float(np.mean(data2))
        std1 = float(np.std(data1))
        std2 = float(np.std(data2))

        # Change calculations
        absolute_change = mean2 - mean1
        if mean1 != 0:
            percent_change = (mean2 - mean1) / abs(mean1) * 100
        else:
            percent_change = 0.0 if mean2 == 0 else 100.0

        # Statistical test (Welch's t-test for unequal variances)
        if len(data1) >= 2 and len(data2) >= 2:
            t_stat, p_value = stats.ttest_ind(data1, data2, equal_var=False)
        else:
            t_stat, p_value = 0.0, 1.0

        # Determine significance
        significant = (
            p_value < self.significance_level and
            abs(percent_change) >= self.significant_change_pct
        )

        # Determine if change is an improvement
        improvement = None
        if higher_is_better is not None and significant:
            if higher_is_better:
                improvement = absolute_change > 0
            else:
                improvement = absolute_change < 0

        return ParameterComparison(
            parameter_name=name,
            session1_mean=mean1,
            session2_mean=mean2,
            session1_std=std1,
            session2_std=std2,
            absolute_change=absolute_change,
            percent_change=percent_change,
            t_statistic=float(t_stat),
            p_value=float(p_value),
            significant=significant,
            improvement=improvement
        )

    def compare_scalar_parameters(
        self,
        session1_params: Dict[str, float],
        session2_params: Dict[str, float]
    ) -> Dict[str, ParameterComparison]:
        """
        Compare scalar parameters (single values) between sessions.

        Args:
            session1_params: Dictionary of parameter values from session 1
            session2_params: Dictionary of parameter values from session 2

        Returns:
            Dictionary of comparison results
        """
        results = {}

        for param_name in set(session1_params.keys()) & set(session2_params.keys()):
            val1 = session1_params[param_name]
            val2 = session2_params[param_name]

            # For scalar values, we can't do t-test, so use simple comparison
            absolute_change = val2 - val1
            if val1 != 0:
                percent_change = (val2 - val1) / abs(val1) * 100
            else:
                percent_change = 0.0 if val2 == 0 else 100.0

            # Get interpretation
            higher_is_better = None
            if param_name in self.PARAMETERS:
                higher_is_better = self.PARAMETERS[param_name][0]

            significant = abs(percent_change) >= self.significant_change_pct

            improvement = None
            if higher_is_better is not None and significant:
                if higher_is_better:
                    improvement = absolute_change > 0
                else:
                    improvement = absolute_change < 0

            results[param_name] = ParameterComparison(
                parameter_name=param_name,
                session1_mean=val1,
                session2_mean=val2,
                session1_std=0.0,
                session2_std=0.0,
                absolute_change=absolute_change,
                percent_change=percent_change,
                t_statistic=0.0,
                p_value=0.0,
                significant=significant,
                improvement=improvement
            )

        return results

    def compare_time_series(
        self,
        session1_data: Dict[str, np.ndarray],
        session2_data: Dict[str, np.ndarray]
    ) -> Dict[str, ParameterComparison]:
        """
        Compare time series data between sessions.

        Args:
            session1_data: Dictionary of arrays from session 1
            session2_data: Dictionary of arrays from session 2

        Returns:
            Dictionary of comparison results
        """
        results = {}

        common_keys = set(session1_data.keys()) & set(session2_data.keys())

        for key in common_keys:
            data1 = session1_data[key]
            data2 = session2_data[key]

            if len(data1) < self.min_samples or len(data2) < self.min_samples:
                continue

            # Get interpretation
            higher_is_better = None
            if key in self.PARAMETERS:
                higher_is_better = self.PARAMETERS[key][0]

            results[key] = self.compare_parameter(
                key, data1, data2, higher_is_better
            )

        return results

    def generate_delta_arrays(
        self,
        session1_data: Dict[str, np.ndarray],
        session2_data: Dict[str, np.ndarray],
        resample_length: int = 1000
    ) -> Dict[str, np.ndarray]:
        """
        Generate delta (difference) arrays for plotting.

        Args:
            session1_data: Session 1 time series
            session2_data: Session 2 time series
            resample_length: Common length for resampling

        Returns:
            Dictionary of delta arrays
        """
        deltas = {}

        common_keys = set(session1_data.keys()) & set(session2_data.keys())

        for key in common_keys:
            data1 = session1_data[key]
            data2 = session2_data[key]

            # Resample to common length
            if len(data1) != resample_length:
                x1 = np.linspace(0, 1, len(data1))
                x_new = np.linspace(0, 1, resample_length)
                data1 = np.interp(x_new, x1, data1)

            if len(data2) != resample_length:
                x2 = np.linspace(0, 1, len(data2))
                x_new = np.linspace(0, 1, resample_length)
                data2 = np.interp(x_new, x2, data2)

            deltas[key] = data2 - data1

        return deltas

    def compare_sessions(
        self,
        session1_id: str,
        session2_id: str,
        session1_data: Dict[str, Any],
        session2_data: Dict[str, Any],
        session1_name: str = "Session 1",
        session2_name: str = "Session 2"
    ) -> ComparisonResult:
        """
        Perform complete session comparison.

        Args:
            session1_id: First session ID
            session2_id: Second session ID
            session1_data: First session data (both scalars and arrays)
            session2_data: Second session data
            session1_name: Display name for session 1
            session2_name: Display name for session 2

        Returns:
            ComparisonResult with all comparisons
        """
        result = ComparisonResult(
            session1_id=session1_id,
            session2_id=session2_id,
            session1_name=session1_name,
            session2_name=session2_name
        )

        # Separate scalars and arrays
        s1_scalars = {}
        s2_scalars = {}
        s1_arrays = {}
        s2_arrays = {}

        for key, value in session1_data.items():
            if isinstance(value, np.ndarray):
                s1_arrays[key] = value
            elif isinstance(value, (int, float)):
                s1_scalars[key] = value

        for key, value in session2_data.items():
            if isinstance(value, np.ndarray):
                s2_arrays[key] = value
            elif isinstance(value, (int, float)):
                s2_scalars[key] = value

        # Compare scalars
        scalar_comparisons = self.compare_scalar_parameters(s1_scalars, s2_scalars)
        result.parameters.update(scalar_comparisons)

        # Compare time series
        array_comparisons = self.compare_time_series(s1_arrays, s2_arrays)
        result.parameters.update(array_comparisons)

        # Categorize results
        for name, comparison in result.parameters.items():
            if comparison.significant:
                if comparison.improvement is True:
                    result.improvements.append(name)
                elif comparison.improvement is False:
                    result.regressions.append(name)
                else:
                    result.neutral.append(name)

        # Calculate overall improvement percentage
        n_compared = len(result.improvements) + len(result.regressions)
        if n_compared > 0:
            result.overall_improvement_pct = len(result.improvements) / n_compared * 100

        return result

    def generate_report(
        self,
        comparison: ComparisonResult
    ) -> str:
        """
        Generate human-readable comparison report.

        Args:
            comparison: ComparisonResult object

        Returns:
            Formatted report string
        """
        lines = []
        lines.append("=" * 60)
        lines.append("SESSION COMPARISON REPORT")
        lines.append("=" * 60)
        lines.append("")
        lines.append(f"Session 1: {comparison.session1_name} ({comparison.session1_id})")
        lines.append(f"Session 2: {comparison.session2_name} ({comparison.session2_id})")
        lines.append("")

        # Summary
        lines.append("-" * 40)
        lines.append("SUMMARY")
        lines.append("-" * 40)
        lines.append(f"Overall improvement: {comparison.overall_improvement_pct:.1f}%")
        lines.append(f"Improvements: {len(comparison.improvements)}")
        lines.append(f"Regressions: {len(comparison.regressions)}")
        lines.append(f"Neutral changes: {len(comparison.neutral)}")
        lines.append("")

        # Improvements
        if comparison.improvements:
            lines.append("-" * 40)
            lines.append("IMPROVEMENTS")
            lines.append("-" * 40)
            for param_name in comparison.improvements:
                comp = comparison.parameters[param_name]
                display_name = self.PARAMETERS.get(param_name, (None, param_name))[1]
                lines.append(f"  {display_name}:")
                lines.append(f"    {comp.session1_mean:.3f} -> {comp.session2_mean:.3f}")
                lines.append(f"    Change: {comp.percent_change:+.1f}%")
            lines.append("")

        # Regressions
        if comparison.regressions:
            lines.append("-" * 40)
            lines.append("REGRESSIONS")
            lines.append("-" * 40)
            for param_name in comparison.regressions:
                comp = comparison.parameters[param_name]
                display_name = self.PARAMETERS.get(param_name, (None, param_name))[1]
                lines.append(f"  {display_name}:")
                lines.append(f"    {comp.session1_mean:.3f} -> {comp.session2_mean:.3f}")
                lines.append(f"    Change: {comp.percent_change:+.1f}%")
            lines.append("")

        # All parameters
        lines.append("-" * 40)
        lines.append("ALL PARAMETERS")
        lines.append("-" * 40)

        for param_name, comp in comparison.parameters.items():
            display_name = self.PARAMETERS.get(param_name, (None, param_name))[1]
            sig = "*" if comp.significant else " "
            lines.append(
                f"{sig} {display_name:30s}: "
                f"{comp.session1_mean:10.3f} -> {comp.session2_mean:10.3f} "
                f"({comp.percent_change:+.1f}%)"
            )

        lines.append("")
        lines.append("* = Statistically significant change")
        lines.append("=" * 60)

        return "\n".join(lines)

    def export_comparison(
        self,
        comparison: ComparisonResult
    ) -> Dict:
        """
        Export comparison result as dictionary for JSON/API.

        Args:
            comparison: ComparisonResult object

        Returns:
            Dictionary representation
        """
        return {
            'session1': {
                'id': comparison.session1_id,
                'name': comparison.session1_name
            },
            'session2': {
                'id': comparison.session2_id,
                'name': comparison.session2_name
            },
            'summary': {
                'overall_improvement_pct': comparison.overall_improvement_pct,
                'improvements_count': len(comparison.improvements),
                'regressions_count': len(comparison.regressions),
                'neutral_count': len(comparison.neutral)
            },
            'improvements': comparison.improvements,
            'regressions': comparison.regressions,
            'neutral': comparison.neutral,
            'parameters': {
                name: {
                    'session1_mean': comp.session1_mean,
                    'session2_mean': comp.session2_mean,
                    'absolute_change': comp.absolute_change,
                    'percent_change': comp.percent_change,
                    'significant': comp.significant,
                    'improvement': comp.improvement
                }
                for name, comp in comparison.parameters.items()
            }
        }
