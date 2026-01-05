"""
Roll Center Analysis Module

Calculates roll centers, roll axis, and analyzes their effect on vehicle dynamics.

The roll center is one of the most important suspension geometry parameters,
affecting load transfer distribution, roll stiffness, and handling balance.

References:
- Milliken & Milliken RCVD Chapter 16-17
- Dixon "Suspension Geometry" Chapter 4
- Staniforth "Competition Car Suspension" Chapter 3
"""

import numpy as np
from typing import Tuple, List, Dict
from dataclasses import dataclass

from .geometry import Point3D, Line3D
from .kinematics import InstantCenter, RollCenter, calculate_roll_center_front_view


@dataclass
class RollAxis:
    """
    Roll axis: line connecting front and rear roll centers.

    The roll axis angle affects:
    - Lateral weight transfer distribution
    - Roll stiffness
    - Jacking forces
    """
    front_rc: RollCenter
    rear_rc: RollCenter
    angle: float  # Degrees from horizontal (positive = front higher)

    @property
    def front_height(self) -> float:
        return self.front_rc.height

    @property
    def rear_height(self) -> float:
        return self.rear_rc.height

    def height_at_cg(self, cg_from_front_axle: float, wheelbase: float) -> float:
        """
        Calculate roll axis height at the center of gravity location.

        This is used for load transfer calculations.

        Args:
            cg_from_front_axle: Distance from front axle to CG (inches)
            wheelbase: Vehicle wheelbase (inches)

        Returns:
            Roll axis height at CG (inches)
        """
        # Linear interpolation along wheelbase
        ratio = cg_from_front_axle / wheelbase
        height = self.front_height + ratio * (self.rear_height - self.front_height)
        return height

    def __repr__(self) -> str:
        return f"RollAxis(front={self.front_height:.2f}\", rear={self.rear_height:.2f}\", angle={self.angle:.2f}°)"


def analyze_roll_center_migration(
    suspension_positions: List[str],
    roll_centers: List[RollCenter]
) -> Dict:
    """
    Analyze how roll center moves with suspension travel.

    Roll center migration is important because:
    - RC height affects jacking forces
    - Large RC migration can cause unpredictable handling
    - Excessive RC rise in bump can cause wheel hop

    Args:
        suspension_positions: List of position names (e.g., ["droop", "ride", "bump"])
        roll_centers: List of RollCenter objects at each position

    Returns:
        Dictionary with migration analysis
    """
    if len(suspension_positions) != len(roll_centers):
        raise ValueError("Position and RC lists must have same length")

    # Calculate RC height range
    heights = [rc.height for rc in roll_centers]
    height_range = max(heights) - min(heights)

    # Calculate lateral migration
    lateral_offsets = [rc.lateral_offset for rc in roll_centers]
    lateral_range = max(lateral_offsets) - min(lateral_offsets)

    # Calculate rates of change
    height_rate = height_range / len(heights) if len(heights) > 1 else 0.0
    lateral_rate = lateral_range / len(heights) if len(heights) > 1 else 0.0

    # Assess migration severity
    if height_range < 1.0:
        height_severity = "Excellent"
    elif height_range < 2.0:
        height_severity = "Good"
    elif height_range < 3.0:
        height_severity = "Acceptable"
    else:
        height_severity = "Poor - consider geometry modifications"

    return {
        'height_range': height_range,
        'lateral_range': lateral_range,
        'min_height': min(heights),
        'max_height': max(heights),
        'height_rate': height_rate,
        'lateral_rate': lateral_rate,
        'assessment': {
            'height_migration': height_severity,
            'notes': []
        }
    }


def calculate_roll_axis(
    front_rc: RollCenter,
    rear_rc: RollCenter,
    wheelbase: float
) -> RollAxis:
    """
    Calculate roll axis from front and rear roll centers.

    The roll axis is the line about which the sprung mass rolls.
    Its height and angle significantly affect handling.

    Typical roll axis angles:
    - Front higher (positive): Common, promotes understeer
    - Level (0°): Neutral
    - Rear higher (negative): Promotes oversteer

    Args:
        front_rc: Front roll center
        rear_rc: Rear roll center
        wheelbase: Vehicle wheelbase (inches)

    Returns:
        RollAxis object
    """
    # Calculate angle from horizontal
    height_diff = front_rc.height - rear_rc.height
    angle_rad = np.arctan2(height_diff, wheelbase)
    angle_deg = np.degrees(angle_rad)

    return RollAxis(
        front_rc=front_rc,
        rear_rc=rear_rc,
        angle=angle_deg
    )


def calculate_jacking_force(
    lateral_acceleration_g: float,
    sprung_weight: float,
    roll_center_height: float,
    track_width: float
) -> float:
    """
    Calculate jacking force: vertical force due to lateral load transfer
    acting through roll center.

    Theory:
    When the car corners, lateral force acts at the CG but is reacted
    at the roll center. If RC is above ground, this creates a vertical
    couple that tries to lift/compress the suspension (jacking).

    Formula (from Milliken RCVD):
    F_jack = (m × ay × h_RC) / (track/2)

    Where:
    - m = sprung mass
    - ay = lateral acceleration
    - h_RC = roll center height
    - track = track width

    Positive jacking = lifting force (undesirable)
    Negative jacking = compression force

    High roll centers create more jacking, which can:
    - Reduce tire loading in corners
    - Cause unpredictable handling
    - Lead to wheel hop

    Args:
        lateral_acceleration_g: Lateral acceleration (g's)
        sprung_weight: Sprung weight (lbs)
        roll_center_height: Roll center height (inches)
        track_width: Track width (inches)

    Returns:
        Jacking force (lbs, positive = lifting)
    """
    # Convert weight to mass (for dimensional analysis)
    # F_lateral = m × a = (W/g) × ay
    lateral_force = sprung_weight * lateral_acceleration_g

    # Jacking force from moment arm
    # Moment = F × h_RC = F_jack × (track/2)
    jacking_force = (lateral_force * roll_center_height) / (track_width / 2)

    return jacking_force


def evaluate_roll_center_height(
    rc_height: float,
    cg_height: float,
    track_width: float,
    axle: str = "front"
) -> Dict:
    """
    Evaluate if roll center height is appropriate.

    Guidelines (from Dixon and Milliken):
    - RC height should be 0-15% of track width above ground
    - RC too low: Excessive body roll, slow response
    - RC too high: Jacking forces, unpredictable handling
    - RC below ground: Can work but causes unique effects

    Front RC typically 1-4" above ground for street cars
    Rear RC typically 2-6" above ground

    Args:
        rc_height: Roll center height (inches)
        cg_height: Center of gravity height (inches)
        track_width: Track width (inches)
        axle: "front" or "rear"

    Returns:
        Dictionary with evaluation
    """
    # Calculate RC height as percentage of track width
    rc_percent = (rc_height / track_width) * 100

    # Distance from CG to RC (affects roll stiffness)
    cg_to_rc = cg_height - rc_height

    # Evaluate height
    if axle == "front":
        ideal_range = (1.0, 4.0)  # inches
    else:  # rear
        ideal_range = (2.0, 6.0)  # inches

    if ideal_range[0] <= rc_height <= ideal_range[1]:
        assessment = "Optimal"
        notes = f"RC height is within ideal range for {axle} suspension"
    elif rc_height < ideal_range[0]:
        assessment = "Low"
        notes = f"RC height is low - expect higher body roll but stable behavior"
    elif rc_height < 0:
        assessment = "Below Ground"
        notes = f"RC is below ground - uncommon but can work in some designs"
    elif rc_height > ideal_range[1] * 1.5:
        assessment = "Too High"
        notes = f"RC height is excessive - expect jacking forces and instability"
    else:
        assessment = "Acceptable"
        notes = f"RC height is slightly high but acceptable"

    return {
        'height': rc_height,
        'height_percent_of_track': rc_percent,
        'distance_from_cg': cg_to_rc,
        'assessment': assessment,
        'notes': notes,
        'ideal_range': ideal_range
    }


if __name__ == "__main__":
    print("Roll Center Analysis Module\n")

    # Example: Calculate roll axis
    front_rc = RollCenter(Point3D(0, 0.1, 3.2), height=3.2, lateral_offset=0.1)
    rear_rc = RollCenter(Point3D(99.9, -0.2, 4.8), height=4.8, lateral_offset=-0.2)

    roll_axis = calculate_roll_axis(front_rc, rear_rc, wheelbase=99.9)
    print(f"Roll Axis: {roll_axis}")

    # Calculate height at CG
    height_at_cg = roll_axis.height_at_cg(cg_from_front_axle=48.2, wheelbase=99.9)
    print(f"Roll axis height at CG: {height_at_cg:.2f}\"")

    # Evaluate front RC
    evaluation = evaluate_roll_center_height(
        rc_height=3.2,
        cg_height=19.5,
        track_width=58.1,
        axle="front"
    )
    print(f"\nFront RC Evaluation: {evaluation['assessment']}")
    print(f"Notes: {evaluation['notes']}")

    # Calculate jacking force
    jacking = calculate_jacking_force(
        lateral_acceleration_g=1.0,
        sprung_weight=2400,
        roll_center_height=3.2,
        track_width=58.1
    )
    print(f"\nJacking force at 1.0g lateral: {jacking:.1f} lbs")
