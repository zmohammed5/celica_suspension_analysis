"""
Core Geometric Primitives for Suspension Analysis

This module provides fundamental geometric operations for 3D suspension
kinematics including points, lines, planes, and their intersections.

Mathematical Reference:
- Vector operations from Milliken & Milliken "Race Car Vehicle Dynamics"
- Computational geometry from O'Rourke "Computational Geometry in C"
"""

import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass


@dataclass
class Point3D:
    """
    Represents a 3D point in vehicle coordinates.

    Coordinate System:
        X: Longitudinal (forward positive)
        Y: Lateral (right positive, driver's perspective)
        Z: Vertical (up positive)
    """
    x: float
    y: float
    z: float

    def to_array(self) -> np.ndarray:
        """Convert to numpy array"""
        return np.array([self.x, self.y, self.z])

    def distance_to(self, other: 'Point3D') -> float:
        """Calculate Euclidean distance to another point"""
        return np.linalg.norm(self.to_array() - other.to_array())

    def __repr__(self) -> str:
        return f"Point3D(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"


@dataclass
class Line3D:
    """
    Represents a 3D line defined by a point and direction vector.

    Parametric form: P(t) = point + t * direction
    where t is the parameter (-∞ to +∞)
    """
    point: Point3D
    direction: np.ndarray  # Unit vector

    def __post_init__(self):
        """Normalize direction vector"""
        self.direction = self.direction / np.linalg.norm(self.direction)

    def point_at(self, t: float) -> Point3D:
        """Get point on line at parameter t"""
        p = self.point.to_array() + t * self.direction
        return Point3D(p[0], p[1], p[2])

    def distance_to_point(self, point: Point3D) -> float:
        """
        Calculate minimum distance from point to line.

        Formula: d = ||(P - P0) × v|| / ||v||
        where P is the point, P0 is a point on the line, v is direction
        """
        p0 = self.point.to_array()
        p = point.to_array()
        cross = np.cross(p - p0, self.direction)
        return np.linalg.norm(cross)


def line_from_two_points(p1: Point3D, p2: Point3D) -> Line3D:
    """
    Create a line passing through two points.

    Args:
        p1: First point
        p2: Second point

    Returns:
        Line3D object
    """
    direction = p2.to_array() - p1.to_array()
    return Line3D(p1, direction)


def intersect_lines_2d(line1: Line3D, line2: Line3D, plane: str = 'xy') -> Optional[Point3D]:
    """
    Find intersection of two lines projected onto a 2D plane.

    This is used for finding instant centers and roll centers which are
    typically calculated in the front view (YZ plane) or side view (XZ plane).

    Mathematical Method:
        For two lines in parametric form:
        L1: P1 + t*d1
        L2: P2 + s*d2

        At intersection: P1 + t*d1 = P2 + s*d2
        Solving for t and s gives the intersection point

    Args:
        line1: First line
        line2: Second line
        plane: Projection plane ('xy', 'xz', or 'yz')

    Returns:
        Intersection point or None if lines are parallel
    """
    # Extract relevant coordinates based on plane
    if plane == 'yz':  # Front view
        idx1, idx2 = 1, 2  # Y and Z
    elif plane == 'xz':  # Side view
        idx1, idx2 = 0, 2  # X and Z
    else:  # xy - Top view
        idx1, idx2 = 0, 1  # X and Y

    # Get 2D projections
    p1 = np.array([line1.point.to_array()[idx1], line1.point.to_array()[idx2]])
    d1 = np.array([line1.direction[idx1], line1.direction[idx2]])

    p2 = np.array([line2.point.to_array()[idx1], line2.point.to_array()[idx2]])
    d2 = np.array([line2.direction[idx1], line2.direction[idx2]])

    # Check if lines are parallel (cross product near zero)
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < 1e-10:
        return None  # Lines are parallel

    # Solve for parameter t
    # (p1 + t*d1) = (p2 + s*d2)
    # p1 - p2 = s*d2 - t*d1
    dp = p2 - p1
    t = (dp[0] * d2[1] - dp[1] * d2[0]) / cross

    # Calculate 3D intersection point
    intersection_3d = line1.point.to_array() + t * line1.direction

    return Point3D(intersection_3d[0], intersection_3d[1], intersection_3d[2])


def angle_between_vectors(v1: np.ndarray, v2: np.ndarray, degrees: bool = True) -> float:
    """
    Calculate angle between two vectors.

    Formula: θ = arccos((v1 · v2) / (|v1| |v2|))

    Args:
        v1: First vector
        v2: Second vector
        degrees: Return angle in degrees (True) or radians (False)

    Returns:
        Angle between vectors
    """
    v1_norm = v1 / np.linalg.norm(v1)
    v2_norm = v2 / np.linalg.norm(v2)

    dot_product = np.dot(v1_norm, v2_norm)
    # Clamp to [-1, 1] to handle numerical errors
    dot_product = np.clip(dot_product, -1.0, 1.0)

    angle_rad = np.arccos(dot_product)

    if degrees:
        return np.degrees(angle_rad)
    return angle_rad


def angle_from_horizontal(vector: np.ndarray, plane: str = 'xz', degrees: bool = True) -> float:
    """
    Calculate angle of a vector from horizontal in a given plane.

    Args:
        vector: 3D vector
        plane: Reference plane ('xz' for side view, 'yz' for front view)
        degrees: Return in degrees (True) or radians (False)

    Returns:
        Angle from horizontal (positive = above horizontal)
    """
    if plane == 'xz':
        # Side view: angle in X-Z plane
        horizontal_component = vector[0]
        vertical_component = vector[2]
    elif plane == 'yz':
        # Front view: angle in Y-Z plane
        horizontal_component = vector[1]
        vertical_component = vector[2]
    else:
        raise ValueError(f"Unknown plane: {plane}")

    angle_rad = np.arctan2(vertical_component, horizontal_component)

    if degrees:
        return np.degrees(angle_rad)
    return angle_rad


def project_point_onto_line(point: Point3D, line: Line3D) -> Point3D:
    """
    Project a point onto a line (find closest point on line).

    Formula: P_proj = P0 + ((P - P0) · v) * v
    where P0 is a point on the line, v is the direction, P is the point

    Args:
        point: Point to project
        line: Line to project onto

    Returns:
        Projected point on the line
    """
    p0 = line.point.to_array()
    p = point.to_array()
    v = line.direction

    # Calculate parameter t for closest point
    t = np.dot(p - p0, v)

    # Calculate projected point
    proj = p0 + t * v

    return Point3D(proj[0], proj[1], proj[2])


def line_plane_intersection(line: Line3D, plane_z: float) -> Optional[Point3D]:
    """
    Find intersection of a line with a horizontal plane (constant Z).

    This is used extensively for finding where suspension lines intersect
    the ground plane (Z=0) or other horizontal reference planes.

    Parametric line: P(t) = P0 + t*d
    Plane: Z = plane_z

    Solving: P0.z + t*d.z = plane_z
             t = (plane_z - P0.z) / d.z

    Args:
        line: Line to intersect
        plane_z: Z coordinate of horizontal plane

    Returns:
        Intersection point or None if line is parallel to plane
    """
    if abs(line.direction[2]) < 1e-10:
        return None  # Line is parallel to plane

    t = (plane_z - line.point.z) / line.direction[2]
    return line.point_at(t)


def calculate_scrub_radius(steering_axis: Line3D, contact_patch: Point3D) -> float:
    """
    Calculate scrub radius: lateral offset between steering axis and
    contact patch at ground level.

    Positive scrub = steering axis outboard of contact patch
    Negative scrub = steering axis inboard of contact patch (desirable)

    Formula: scrub = Y_steering_axis(Z=0) - Y_contact_patch

    Args:
        steering_axis: Line representing the steering axis (from strut top through ball joint)
        contact_patch: Tire contact patch point

    Returns:
        Scrub radius in inches (positive = outboard)
    """
    # Find where steering axis intersects ground (Z=0)
    ground_point = line_plane_intersection(steering_axis, 0.0)

    if ground_point is None:
        raise ValueError("Steering axis is parallel to ground (impossible)")

    # Scrub radius is lateral offset
    scrub = ground_point.y - contact_patch.y

    return scrub


def midpoint(p1: Point3D, p2: Point3D) -> Point3D:
    """Calculate midpoint between two points"""
    mid = (p1.to_array() + p2.to_array()) / 2
    return Point3D(mid[0], mid[1], mid[2])


if __name__ == "__main__":
    # Example usage and testing
    print("Testing Geometric Primitives\n")

    # Test points
    p1 = Point3D(0, 0, 0)
    p2 = Point3D(3, 4, 0)
    print(f"Distance from {p1} to {p2}: {p1.distance_to(p2):.3f} inches")

    # Test line creation
    line = line_from_two_points(p1, p2)
    print(f"\nLine from origin to (3,4,0):")
    print(f"  Direction: {line.direction}")

    # Test angle calculation
    v1 = np.array([1, 0, 0])
    v2 = np.array([0, 1, 0])
    angle = angle_between_vectors(v1, v2)
    print(f"\nAngle between [1,0,0] and [0,1,0]: {angle:.1f}°")
