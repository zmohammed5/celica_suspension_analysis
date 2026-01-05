"""
Suspension Kinematics Calculator

Core algorithms for calculating suspension kinematics parameters including:
- Instant centers
- Roll centers
- Camber gain
- Motion ratios
- Anti-geometry (anti-dive, anti-squat)

Mathematical References:
- Milliken & Milliken "Race Car Vehicle Dynamics" Chapter 16-17
- Gillespie "Fundamentals of Vehicle Dynamics" Chapter 7
- Dixon "Suspension Geometry and Computation" Chapter 3-5
"""

import numpy as np
from typing import Dict, Tuple, List, Optional
from dataclasses import dataclass, field

from .geometry import (
    Point3D, Line3D, line_from_two_points, intersect_lines_2d,
    line_plane_intersection, angle_from_horizontal, angle_between_vectors
)


@dataclass
class InstantCenter:
    """
    Instant center of rotation for a suspension linkage.

    The instant center is the point about which the wheel assembly
    instantaneously rotates in a given plane projection.
    """
    point: Point3D
    description: str = ""

    def __repr__(self) -> str:
        return f"IC({self.point.y:.2f}, {self.point.z:.2f}) {self.description}"


@dataclass
class RollCenter:
    """
    Roll center: the point about which the sprung mass rolls.

    The roll center height significantly affects:
    - Load transfer distribution
    - Roll stiffness
    - Handling balance
    """
    point: Point3D
    height: float  # Z coordinate (height above ground)
    lateral_offset: float  # Y coordinate (offset from centerline)

    def __repr__(self) -> str:
        return f"RC(Y={self.lateral_offset:.2f}\", Z={self.height:.2f}\")"


@dataclass
class KinematicAnalysis:
    """Results of complete kinematic analysis for one axle"""
    roll_center: RollCenter
    instant_centers: Dict[str, InstantCenter] = field(default_factory=dict)
    swing_arm_length: float = 0.0
    roll_center_swing_arm_angle: float = 0.0

    # Camber characteristics
    static_camber: float = 0.0
    camber_gain: float = 0.0  # degrees per inch of travel

    # Geometry angles
    control_arm_angle: float = 0.0  # From horizontal
    strut_angle: float = 0.0  # From vertical

    # Motion ratio
    motion_ratio: float = 0.0

    # Additional metrics
    scrub_radius: float = 0.0
    kingpin_inclination: float = 0.0
    caster_angle: float = 0.0


def calculate_macpherson_instant_center(
    strut_top: Point3D,
    ball_joint: Point3D,
    lca_inner_front: Point3D,
    lca_inner_rear: Point3D,
    side: str = "left"
) -> InstantCenter:
    """
    Calculate instant center for MacPherson strut suspension.

    Theory:
    The instant center is found by extending two lines:
    1. The strut axis (from top mount through ball joint)
    2. The lower control arm axis (through inner pivot points)

    The intersection of these lines (in front view projection) is the
    instant center about which the wheel instantaneously rotates.

    Mathematical Method:
    - Create line through strut mount and ball joint
    - Create line through LCA inner pivots
    - Find intersection in Y-Z plane (front view)

    Reference: Milliken & Milliken, Section 16.2

    Args:
        strut_top: Strut top mount point
        ball_joint: Ball joint location
        lca_inner_front: Front inner pivot of lower control arm
        lca_inner_rear: Rear inner pivot of lower control arm
        side: "left" or "right"

    Returns:
        InstantCenter object
    """
    # Line 1: Strut axis
    strut_line = line_from_two_points(strut_top, ball_joint)

    # Line 2: Lower control arm axis
    # The control arm pivots about a line through its inner mounts
    lca_line = line_from_two_points(lca_inner_front, lca_inner_rear)

    # Find intersection in front view (Y-Z plane)
    ic_point = intersect_lines_2d(strut_line, lca_line, plane='yz')

    if ic_point is None:
        # Lines are parallel - shouldn't happen in real suspension
        raise ValueError(f"Strut and LCA are parallel on {side} side - check measurements")

    return InstantCenter(
        point=ic_point,
        description=f"{side.capitalize()} front instant center"
    )


def calculate_roll_center_front_view(
    left_ic: InstantCenter,
    right_ic: InstantCenter,
    left_contact_patch: Point3D,
    right_contact_patch: Point3D
) -> RollCenter:
    """
    Calculate roll center using instant centers method.

    Theory:
    The roll center is found by:
    1. Drawing a line from each instant center through its respective contact patch
    2. These lines represent the "virtual swing arms"
    3. The intersection of these two lines is the roll center

    This method assumes:
    - Small suspension travel (valid near static position)
    - No significant tire deflection
    - Rigid suspension links

    Formula Development:
    For each side, the wheel moves in an arc about the instant center.
    The instantaneous velocity of the contact patch is perpendicular to
    the line from IC to contact patch. The sprung mass moves perpendicular
    to the line from RC to contact patch. These must be compatible.

    Reference: Milliken & Milliken, Section 16.3-16.4

    Args:
        left_ic: Left side instant center
        right_ic: Right side instant center
        left_contact_patch: Left tire contact patch
        right_contact_patch: Right tire contact patch

    Returns:
        RollCenter object
    """
    # Line from left IC through left contact patch
    left_swing_arm = line_from_two_points(left_ic.point, left_contact_patch)

    # Line from right IC through right contact patch
    right_swing_arm = line_from_two_points(right_ic.point, right_contact_patch)

    # Find intersection in front view (Y-Z plane)
    rc_point = intersect_lines_2d(left_swing_arm, right_swing_arm, plane='yz')

    if rc_point is None:
        # Parallel swing arms = roll center at infinity (very rare)
        # Use a very high roll center as approximation
        rc_point = Point3D(0, 0, 1000.0)

    return RollCenter(
        point=rc_point,
        height=rc_point.z,
        lateral_offset=rc_point.y
    )


def calculate_camber_gain(
    wheel_travel: np.ndarray,
    camber_angles: np.ndarray
) -> float:
    """
    Calculate camber gain: rate of camber change with suspension travel.

    Camber gain is critical for maintaining tire contact patch in corners.
    Typical values:
    - MacPherson strut: 0.8 - 1.5 deg/inch
    - Double wishbone: 0.5 - 1.2 deg/inch

    Positive camber gain = more negative camber in bump (good for cornering)

    Formula: camber_gain = Δcamber / Δtravel

    Args:
        wheel_travel: Array of wheel travel positions (inches from static)
        camber_angles: Array of camber angles at each position (degrees)

    Returns:
        Camber gain in degrees per inch (negative camber gain means
        wheel goes more positive in bump)
    """
    # Linear fit to get average rate
    coefficients = np.polyfit(wheel_travel, camber_angles, 1)
    camber_gain = coefficients[0]  # Slope of the line

    return camber_gain


def calculate_motion_ratio(
    wheel_center_travel: float,
    spring_travel: float
) -> float:
    """
    Calculate motion ratio: spring displacement per wheel displacement.

    Motion ratio affects:
    - Wheel rate (spring_rate × MR²)
    - Damping effectiveness
    - Available suspension travel

    Formula: MR = spring_travel / wheel_travel

    Typical values:
    - MacPherson strut: 0.6 - 0.8 (spring on strut)
    - Pushrod: 0.5 - 0.7
    - Pullrod: 0.5 - 0.7

    Lower MR = softer wheel rate for same spring rate
    Higher MR = more efficient spring/damper use

    Args:
        wheel_center_travel: Vertical wheel center displacement (inches)
        spring_travel: Spring displacement (inches)

    Returns:
        Motion ratio (dimensionless)
    """
    if abs(wheel_center_travel) < 0.001:
        raise ValueError("Wheel travel too small to calculate motion ratio")

    mr = spring_travel / wheel_center_travel
    return abs(mr)


def calculate_anti_dive(
    lca_angle: float,
    wheelbase: float,
    cg_height: float,
    cg_to_front_axle: float
) -> float:
    """
    Calculate anti-dive percentage for front suspension.

    Anti-dive resists nose dive under braking by using braking forces
    to compress the suspension, counteracting weight transfer.

    Theory:
    When braking, the contact patch pushes rearward on the tire.
    This force travels up through the suspension linkage.
    If the control arm is angled upward toward the rear, this
    creates an upward component that opposes dive.

    Formula (from Dixon):
    Anti-dive % = (tan(θ) × wheelbase × 100) / (2 × h_cg)

    Where:
    - θ = LCA angle from horizontal (positive = up toward rear)
    - wheelbase = distance between axles
    - h_cg = center of gravity height

    Typical values:
    - 0%: Pure weight transfer (significant dive)
    - 50%: Half of dive is resisted
    - 100%: Complete anti-dive (no pitch under braking)
    - >100%: Anti-dive jacking (suspension extends under braking)

    Generally 30-60% is desirable for street cars.

    Args:
        lca_angle: Lower control arm angle from horizontal (degrees, + = up toward rear)
        wheelbase: Vehicle wheelbase (inches)
        cg_height: Center of gravity height (inches)
        cg_to_front_axle: Distance from CG to front axle (inches)

    Returns:
        Anti-dive percentage
    """
    # Convert angle to radians
    theta_rad = np.deg2rad(lca_angle)

    # Calculate anti-dive percentage
    # Formula from Dixon "Suspension Geometry and Computation" Eq 3.12
    anti_dive = (np.tan(theta_rad) * wheelbase * 100) / (2 * cg_height)

    return anti_dive


def calculate_anti_squat(
    link_angles: Dict[str, float],
    wheelbase: float,
    cg_height: float,
    cg_to_rear_axle: float,
    weight_distribution_rear: float
) -> float:
    """
    Calculate anti-squat percentage for rear suspension.

    Anti-squat resists squat (rear suspension compression) during
    acceleration by using drivetrain torque to extend the suspension.

    For FWD: Anti-squat is typically low since rear suspension doesn't
    transmit drive forces.

    Formula (simplified for independent rear):
    Anti-squat % = (tan(θ) × wheelbase × 100) / (2 × h_cg)

    Where θ is the effective angle of the rear suspension linkage.

    Args:
        link_angles: Dictionary of suspension link angles
        wheelbase: Vehicle wheelbase (inches)
        cg_height: Center of gravity height (inches)
        cg_to_rear_axle: Distance from CG to rear axle (inches)
        weight_distribution_rear: Rear weight percentage (0-100)

    Returns:
        Anti-squat percentage
    """
    # For multi-link, use average of lower link angles
    # This is a simplification - full calculation requires force analysis
    if 'lower_arm' in link_angles:
        theta = link_angles['lower_arm']
    else:
        theta = 0.0  # Conservative estimate

    theta_rad = np.deg2rad(theta)

    # Simplified anti-squat (assumes longitudinal force through lower link)
    anti_squat = (np.tan(theta_rad) * wheelbase * 100) / (2 * cg_height)

    # Adjust for weight distribution (FWD has low rear weight)
    anti_squat *= (weight_distribution_rear / 100)

    return anti_squat


def analyze_macpherson_strut(
    strut_top: Point3D,
    ball_joint: Point3D,
    lca_inner_front: Point3D,
    lca_inner_rear: Point3D,
    wheel_center: Point3D,
    contact_patch: Point3D,
    side: str = "left"
) -> Dict:
    """
    Complete kinematic analysis of a MacPherson strut corner.

    Args:
        strut_top: Strut top mount location
        ball_joint: Ball joint location
        lca_inner_front: Front inner LCA pivot
        lca_inner_rear: Rear inner LCA pivot
        wheel_center: Wheel center location
        contact_patch: Tire contact patch location
        side: "left" or "right"

    Returns:
        Dictionary with all kinematic parameters
    """
    # Calculate instant center
    ic = calculate_macpherson_instant_center(
        strut_top, ball_joint, lca_inner_front, lca_inner_rear, side
    )

    # Calculate control arm angle
    lca_vector = lca_inner_rear.to_array() - lca_inner_front.to_array()
    lca_angle = angle_from_horizontal(lca_vector, plane='xz')  # Side view

    # Calculate strut angle from vertical
    strut_vector = ball_joint.to_array() - strut_top.to_array()
    vertical = np.array([0, 0, -1])  # Down
    strut_angle = angle_between_vectors(strut_vector, vertical)

    # Swing arm length (IC to contact patch)
    swing_arm_length = ic.point.distance_to(contact_patch)

    # Swing arm angle
    swing_arm_vector = contact_patch.to_array() - ic.point.to_array()
    swing_arm_angle = angle_from_horizontal(swing_arm_vector, plane='yz')

    return {
        'instant_center': ic,
        'lca_angle': lca_angle,
        'strut_angle': strut_angle,
        'swing_arm_length': swing_arm_length,
        'swing_arm_angle': swing_arm_angle
    }


if __name__ == "__main__":
    print("Suspension Kinematics Calculator\n")
    print("Testing with example MacPherson strut geometry...\n")

    # Example geometry (left front)
    strut_top = Point3D(4.2, -24.5, 28.3)
    ball_joint = Point3D(4.8, -26.7, 10.2)
    lca_front = Point3D(10.5, -11.2, 6.8)
    lca_rear = Point3D(-2.3, -11.8, 6.5)
    contact_patch = Point3D(0.0, -29.05, 0.0)

    ic = calculate_macpherson_instant_center(strut_top, ball_joint, lca_front, lca_rear)
    print(f"Instant Center: Y={ic.point.y:.2f}\", Z={ic.point.z:.2f}\"")

    # Test camber gain
    travel = np.array([-3.2, -1.6, 0.0, 1.6, 3.2])
    camber = np.array([0.2, -0.5, -1.2, -1.9, -2.8])
    gain = calculate_camber_gain(travel, camber)
    print(f"\nCamber Gain: {gain:.3f} deg/inch")

    # Test anti-dive
    anti_dive = calculate_anti_dive(8.0, 99.9, 19.5, 48.2)
    print(f"Anti-Dive: {anti_dive:.1f}%")
