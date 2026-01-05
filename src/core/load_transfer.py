"""
Load Transfer Analysis Module

Calculates lateral and longitudinal load transfer, and their distribution
between front and rear axles.

Load transfer is critical for understanding:
- Tire loading in corners
- Understeer/oversteer balance
- Effect of suspension modifications

References:
- Milliken & Milliken RCVD Chapter 6
- Gillespie "Fundamentals of Vehicle Dynamics" Chapter 6
"""

import numpy as np
from typing import Dict, Tuple
from dataclasses import dataclass


@dataclass
class LoadTransferResult:
    """Results of load transfer calculation"""
    total_transfer: float  # Total lateral load transfer (lbs)
    front_transfer: float  # Load transfer at front axle (lbs)
    rear_transfer: float  # Load transfer at rear axle (lbs)
    front_percentage: float  # Percentage of total at front
    rear_percentage: float  # Percentage of total at rear


def calculate_lateral_load_transfer(
    lateral_acceleration_g: float,
    total_weight: float,
    cg_height: float,
    track_width: float
) -> float:
    """
    Calculate total lateral load transfer.

    Theory:
    In a turn, the lateral force at the CG creates a moment about
    the contact patches, transferring load from inside tires to outside tires.

    Formula (from Milliken RCVD Eq 6.1):
    ΔW_lat = (m × ay × h_cg) / t

    Where:
    - m = vehicle mass (W/g)
    - ay = lateral acceleration
    - h_cg = CG height above ground
    - t = track width

    This is the total load transferred from inside wheels to outside wheels.

    Args:
        lateral_acceleration_g: Lateral acceleration (g's)
        total_weight: Total vehicle weight (lbs)
        cg_height: CG height above ground (inches)
        track_width: Average track width (inches)

    Returns:
        Total lateral load transfer (lbs, transferred to outside)
    """
    # ΔW = (W × ay × h) / t
    # Note: W/g cancels to give W directly when ay is in g's
    load_transfer = (total_weight * lateral_acceleration_g * cg_height) / track_width

    return load_transfer


def distribute_load_transfer(
    total_lateral_transfer: float,
    total_weight: float,
    wheelbase: float,
    cg_from_front_axle: float,
    front_track: float,
    rear_track: float,
    front_roll_stiffness: float,
    rear_roll_stiffness: float
) -> LoadTransferResult:
    """
    Distribute lateral load transfer between front and rear axles.

    Theory:
    Load transfer is distributed by TWO mechanisms:

    1. Geometric (Unsprung) Transfer - from CG height and weight distribution
       This portion is proportional to the vertical load on each axle

    2. Elastic (Sprung) Transfer - from body roll acting through suspension
       This portion is proportional to roll stiffness at each axle

    Formula (from Milliken RCVD Section 6.2):

    Geometric transfer split:
    Front_geo = ΔW_lat × (W_rear / W_total)
    Rear_geo = ΔW_lat × (W_front / W_total)

    (Note the cross-relationship: front transfer depends on rear weight!)

    Elastic transfer split:
    Front_elastic / Total_elastic = K_front / (K_front + K_rear)
    Rear_elastic / Total_elastic = K_rear / (K_front + K_rear)

    Where K is roll stiffness at each axle

    Args:
        total_lateral_transfer: Total lateral load transfer (lbs)
        total_weight: Total vehicle weight (lbs)
        wheelbase: Wheelbase (inches)
        cg_from_front_axle: CG distance from front axle (inches)
        front_track: Front track width (inches)
        rear_track: Rear track width (inches)
        front_roll_stiffness: Front roll stiffness (lb-in/deg)
        rear_roll_stiffness: Rear roll stiffness (lb-in/deg)

    Returns:
        LoadTransferResult with distribution breakdown
    """
    # Calculate static weight distribution
    front_weight = total_weight * ((wheelbase - cg_from_front_axle) / wheelbase)
    rear_weight = total_weight - front_weight

    front_weight_pct = (front_weight / total_weight) * 100
    rear_weight_pct = (rear_weight / total_weight) * 100

    # Method 1: Simple geometric distribution (ignores roll stiffness)
    # This is the unsprung load transfer
    front_transfer_geometric = total_lateral_transfer * (rear_weight / total_weight)
    rear_transfer_geometric = total_lateral_transfer * (front_weight / total_weight)

    # Method 2: Include roll stiffness distribution (more accurate)
    total_roll_stiffness = front_roll_stiffness + rear_roll_stiffness

    if total_roll_stiffness > 0:
        # Elastic load transfer distribution
        front_elastic_fraction = front_roll_stiffness / total_roll_stiffness
        rear_elastic_fraction = rear_roll_stiffness / total_roll_stiffness

        # Total transfer = geometric + elastic portions
        # For simplicity, assume 50/50 split between geometric and elastic
        # (more rigorous analysis requires roll angle calculation)
        front_transfer_total = (front_transfer_geometric +
                               total_lateral_transfer * front_elastic_fraction) / 2
        rear_transfer_total = (rear_transfer_geometric +
                              total_lateral_transfer * rear_elastic_fraction) / 2
    else:
        # No roll stiffness data - use geometric only
        front_transfer_total = front_transfer_geometric
        rear_transfer_total = rear_transfer_geometric

    # Calculate percentages
    total_calculated = front_transfer_total + rear_transfer_total
    if total_calculated > 0:
        front_pct = (front_transfer_total / total_calculated) * 100
        rear_pct = (rear_transfer_total / total_calculated) * 100
    else:
        front_pct = 50.0
        rear_pct = 50.0

    return LoadTransferResult(
        total_transfer=total_lateral_transfer,
        front_transfer=front_transfer_total,
        rear_transfer=rear_transfer_total,
        front_percentage=front_pct,
        rear_percentage=rear_pct
    )


def calculate_longitudinal_load_transfer(
    longitudinal_acceleration_g: float,
    total_weight: float,
    cg_height: float,
    wheelbase: float
) -> Tuple[float, float]:
    """
    Calculate longitudinal load transfer (braking or acceleration).

    Theory:
    Longitudinal acceleration creates a moment about the contact patches,
    transferring load from rear to front (braking) or front to rear (acceleration).

    Formula (from Gillespie Eq 6.11):
    ΔW_front = (W × ax × h_cg) / wheelbase
    ΔW_rear = -ΔW_front

    Positive ax = acceleration (load to rear)
    Negative ax = braking (load to front)

    Args:
        longitudinal_acceleration_g: Longitudinal acceleration (g's, + = accel, - = brake)
        total_weight: Total vehicle weight (lbs)
        cg_height: CG height (inches)
        wheelbase: Wheelbase (inches)

    Returns:
        Tuple of (front_transfer, rear_transfer) in lbs
        Positive = load added, negative = load removed
    """
    # Calculate load transfer
    transfer = (total_weight * longitudinal_acceleration_g * cg_height) / wheelbase

    # During braking (negative ax), front gains load (positive)
    # During accel (positive ax), rear gains load (negative on front)
    front_transfer = -transfer  # Negative of acceleration
    rear_transfer = transfer

    return front_transfer, rear_transfer


def estimate_roll_stiffness(
    spring_rate: float,
    motion_ratio: float,
    track_width: float,
    anti_roll_bar_rate: float = 0.0
) -> float:
    """
    Estimate roll stiffness at one axle.

    Roll stiffness combines spring stiffness and anti-roll bar stiffness.

    Formula (from Milliken RCVD):
    K_roll = (K_spring × MR² × t²) / 2 + K_arb

    Where:
    - K_spring = spring rate (lbs/in)
    - MR = motion ratio (dimensionless)
    - t = track width (inches)
    - K_arb = anti-roll bar rate (lb-in/deg)

    Args:
        spring_rate: Spring rate (lbs/in)
        motion_ratio: Motion ratio (spring travel / wheel travel)
        track_width: Track width (inches)
        anti_roll_bar_rate: Anti-roll bar rate (lb-in/deg, optional)

    Returns:
        Roll stiffness (lb-in/deg)
    """
    # Spring contribution to roll stiffness
    # Convert from lbs/in to lb-in/deg (involves geometry)
    # Approximate conversion: 1 degree roll ≈ track/100 inches of travel at wheel
    wheel_rate = spring_rate * motion_ratio**2
    spring_roll_stiffness = (wheel_rate * track_width**2) / (2 * 57.3)  # 57.3 = rad to deg

    # Total roll stiffness
    total_roll_stiffness = spring_roll_stiffness + anti_roll_bar_rate

    return total_roll_stiffness


def analyze_handling_balance(
    front_lateral_transfer_pct: float,
    rear_lateral_transfer_pct: float,
    front_weight_pct: float
) -> Dict:
    """
    Analyze understeer/oversteer tendency from load transfer distribution.

    Theory:
    The axle with higher load transfer (relative to its static load) will
    have more tire saturation and thus lose grip first.

    Understeer: Front loses grip first (front-biased load transfer)
    Oversteer: Rear loses grip first (rear-biased load transfer)
    Neutral: Balanced load transfer relative to weight distribution

    Rule of thumb (Dixon):
    - Ideal lateral load transfer distribution ≈ static weight distribution
    - Front-biased LT → understeer
    - Rear-biased LT → oversteer

    Args:
        front_lateral_transfer_pct: Front lateral load transfer (% of total)
        rear_lateral_transfer_pct: Rear lateral load transfer (% of total)
        front_weight_pct: Static front weight (% of total)

    Returns:
        Dictionary with handling balance analysis
    """
    # Calculate bias: difference between LT distribution and weight distribution
    front_bias = front_lateral_transfer_pct - front_weight_pct

    if abs(front_bias) < 2.0:
        balance = "Neutral"
        tendency = "Balanced handling - front and rear will reach limit together"
    elif front_bias > 5.0:
        balance = "Understeer"
        tendency = f"Front-biased load transfer ({front_bias:+.1f}%) promotes understeer"
    elif front_bias < -5.0:
        balance = "Oversteer"
        tendency = f"Rear-biased load transfer ({front_bias:+.1f}%) promotes oversteer"
    elif front_bias > 0:
        balance = "Mild Understeer"
        tendency = f"Slight front bias ({front_bias:+.1f}%) - mild understeer tendency"
    else:
        balance = "Mild Oversteer"
        tendency = f"Slight rear bias ({front_bias:+.1f}%) - mild oversteer tendency"

    return {
        'balance': balance,
        'front_bias': front_bias,
        'tendency': tendency,
        'recommendations': _get_balance_recommendations(balance, front_bias)
    }


def _get_balance_recommendations(balance: str, front_bias: float) -> List[str]:
    """Generate recommendations for adjusting handling balance"""
    recommendations = []

    if "Understeer" in balance and front_bias > 3.0:
        recommendations.append("Reduce front roll stiffness or increase rear")
        recommendations.append("Add rear anti-roll bar or remove/reduce front")
        recommendations.append("Soften front springs or stiffen rear springs")
        recommendations.append("Lower front roll center or raise rear (if safe)")
    elif "Oversteer" in balance and front_bias < -3.0:
        recommendations.append("Increase front roll stiffness or reduce rear")
        recommendations.append("Add front anti-roll bar or remove/reduce rear")
        recommendations.append("Stiffen front springs or soften rear springs")
        recommendations.append("Raise front roll center or lower rear (if safe)")
    else:
        recommendations.append("Current balance is acceptable")
        recommendations.append("Fine-tune with tire pressures and alignment")

    return recommendations


if __name__ == "__main__":
    print("Load Transfer Analysis Module\n")

    # Example: 1992 Celica GT in 1.0g corner
    total_transfer = calculate_lateral_load_transfer(
        lateral_acceleration_g=1.0,
        total_weight=2690,
        cg_height=19.5,
        track_width=58.0
    )
    print(f"Total lateral load transfer at 1.0g: {total_transfer:.1f} lbs\n")

    # Estimate roll stiffness
    front_k_roll = estimate_roll_stiffness(180, 0.7, 58.1)
    rear_k_roll = estimate_roll_stiffness(160, 0.65, 57.9)
    print(f"Front roll stiffness: {front_k_roll:.0f} lb-in/deg")
    print(f"Rear roll stiffness: {rear_k_roll:.0f} lb-in/deg\n")

    # Distribute load transfer
    result = distribute_load_transfer(
        total_lateral_transfer=total_transfer,
        total_weight=2690,
        wheelbase=99.9,
        cg_from_front_axle=48.2,
        front_track=58.1,
        rear_track=57.9,
        front_roll_stiffness=front_k_roll,
        rear_roll_stiffness=rear_k_roll
    )

    print(f"Front lateral LT: {result.front_transfer:.1f} lbs ({result.front_percentage:.1f}%)")
    print(f"Rear lateral LT: {result.rear_transfer:.1f} lbs ({result.rear_percentage:.1f}%)\n")

    # Analyze balance
    balance = analyze_handling_balance(
        front_lateral_transfer_pct=result.front_percentage,
        rear_lateral_transfer_pct=result.rear_percentage,
        front_weight_pct=61.5
    )
    print(f"Handling Balance: {balance['balance']}")
    print(f"Tendency: {balance['tendency']}")
