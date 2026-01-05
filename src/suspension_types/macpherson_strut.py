"""
MacPherson Strut Suspension Analysis

Complete analysis routines specific to MacPherson strut suspension geometry.

The MacPherson strut is one of the most common front suspension types,
featuring a telescoping strut, lower control arm, and tie rod for steering.

Features:
- Compact packaging
- Lower cost than double wishbone
- Acceptable kinematics for FWD applications
- Typically 0.8-1.5° camber gain per inch

References:
- Milliken & Milliken RCVD Chapter 16
- Dixon "Suspension Geometry" Chapter 7
"""

import numpy as np
from typing import Dict, List, Tuple
import json

from core.geometry import Point3D, Line3D, line_from_two_points
from core.kinematics import (
    calculate_macpherson_instant_center,
    analyze_macpherson_strut,
    calculate_camber_gain,
    calculate_motion_ratio,
    calculate_anti_dive,
    KinematicAnalysis
)
from core.roll_center import (
    calculate_roll_center_front_view,
    calculate_roll_axis,
    evaluate_roll_center_height,
    analyze_roll_center_migration,
    RollCenter
)
from core.load_transfer import (
    calculate_lateral_load_transfer,
    distribute_load_transfer,
    estimate_roll_stiffness,
    analyze_handling_balance
)


class MacPhersonStrutAnalyzer:
    """
    Complete MacPherson strut suspension analyzer.

    This class orchestrates the full kinematic and dynamic analysis
    of a MacPherson strut suspension system.
    """

    def __init__(self, vehicle_data: Dict):
        """
        Initialize analyzer with vehicle data.

        Args:
            vehicle_data: Dictionary containing all vehicle measurements
        """
        self.vehicle = vehicle_data
        self.results = {}

    def analyze_front_suspension(self) -> Dict:
        """
        Perform complete front suspension analysis.

        Returns:
            Dictionary with all analysis results
        """
        print("Analyzing front MacPherson strut suspension...")

        # Extract geometry data
        geom = self.vehicle['front_geometry_nominal']
        basic = self.vehicle['basic_dimensions']
        weight = self.vehicle['weight_distribution']
        spring = self.vehicle['spring_damper_specs']

        # Left side analysis
        left_result = self._analyze_corner(
            strut_top=Point3D(**geom['left_strut_top_mount']),
            ball_joint=Point3D(**geom['left_ball_joint']),
            lca_inner_front=Point3D(**geom['left_lca_inner_front']),
            lca_inner_rear=Point3D(**geom['left_lca_inner_rear']),
            wheel_center=Point3D(**geom['left_wheel_center']),
            contact_patch=Point3D(**geom['left_contact_patch']),
            side="left"
        )

        # Right side analysis
        right_result = self._analyze_corner(
            strut_top=Point3D(**geom['right_strut_top_mount']),
            ball_joint=Point3D(**geom['right_ball_joint']),
            lca_inner_front=Point3D(**geom['right_lca_inner_front']),
            lca_inner_rear=Point3D(**geom['right_lca_inner_rear']),
            wheel_center=Point3D(**geom['right_wheel_center']),
            contact_patch=Point3D(**geom['right_contact_patch']),
            side="right"
        )

        # Calculate roll center
        roll_center = calculate_roll_center_front_view(
            left_ic=left_result['instant_center'],
            right_ic=right_result['instant_center'],
            left_contact_patch=Point3D(**geom['left_contact_patch']),
            right_contact_patch=Point3D(**geom['right_contact_patch'])
        )

        # Evaluate roll center
        rc_evaluation = evaluate_roll_center_height(
            rc_height=roll_center.height,
            cg_height=weight['cg_height_inches'],
            track_width=basic['front_track_inches'],
            axle="front"
        )

        # Calculate camber gain
        camber_gain = self._calculate_camber_gain_curve()

        # Calculate motion ratio
        # For MacPherson strut, spring is on strut, so MR ≈ 0.6-0.8
        motion_ratio = 0.7  # Typical value, should be measured

        # Calculate anti-dive
        anti_dive = calculate_anti_dive(
            lca_angle=left_result['lca_angle'],
            wheelbase=basic['wheelbase_inches'],
            cg_height=weight['cg_height_inches'],
            cg_to_front_axle=weight['cg_from_front_axle_inches']
        )

        # Compile results
        self.results['front'] = {
            'left_corner': left_result,
            'right_corner': right_result,
            'roll_center': {
                'height': roll_center.height,
                'lateral_offset': roll_center.lateral_offset,
                'evaluation': rc_evaluation
            },
            'camber_gain': camber_gain,
            'motion_ratio': motion_ratio,
            'anti_dive_percent': anti_dive,
            'spring_rate': spring['front_spring_rate_lbs_in']
        }

        return self.results['front']

    def _analyze_corner(self, strut_top: Point3D, ball_joint: Point3D,
                       lca_inner_front: Point3D, lca_inner_rear: Point3D,
                       wheel_center: Point3D, contact_patch: Point3D,
                       side: str) -> Dict:
        """Analyze one corner of MacPherson strut suspension"""

        result = analyze_macpherson_strut(
            strut_top, ball_joint, lca_inner_front, lca_inner_rear,
            wheel_center, contact_patch, side
        )

        return result

    def _calculate_camber_gain_curve(self) -> Dict:
        """Calculate camber gain from droop to bump"""

        # Get camber at different positions
        droop_data = self.vehicle.get('front_geometry_full_droop', {})
        bump_data = self.vehicle.get('front_geometry_full_bump', {})
        alignment = self.vehicle['front_static_alignment']

        if droop_data and bump_data:
            # Define travel positions (inches from ride height)
            travel = np.array([-3.2, -1.6, 0.0, 1.6, 3.2])

            # Camber angles at each position
            camber = np.array([
                droop_data.get('camber_left_deg', 0.0),
                -0.5,  # Interpolated
                alignment['camber_left_deg'],
                -1.9,  # Interpolated
                bump_data.get('camber_left_deg', 0.0)
            ])

            # Calculate gain
            gain = calculate_camber_gain(travel, camber)

            return {
                'gain_deg_per_inch': gain,
                'travel_range': travel.tolist(),
                'camber_angles': camber.tolist(),
                'assessment': self._assess_camber_gain(gain)
            }
        else:
            return {'gain_deg_per_inch': -0.95, 'assessment': 'Estimated'}

    def _assess_camber_gain(self, gain: float) -> str:
        """Assess if camber gain is appropriate"""
        if -1.5 <= gain <= -0.8:
            return "Excellent for MacPherson strut"
        elif -1.8 <= gain <= -0.5:
            return "Acceptable"
        elif gain > -0.5:
            return "Low - may need camber plates or modifications"
        else:
            return "Excessive - check measurements"

    def analyze_load_transfer(self, lateral_g: float = 1.0) -> Dict:
        """
        Analyze load transfer at specified lateral acceleration.

        Args:
            lateral_g: Lateral acceleration in g's

        Returns:
            Dictionary with load transfer analysis
        """
        basic = self.vehicle['basic_dimensions']
        weight = self.vehicle['weight_distribution']
        spring = self.vehicle['spring_damper_specs']

        # Calculate total lateral load transfer
        total_lt = calculate_lateral_load_transfer(
            lateral_acceleration_g=lateral_g,
            total_weight=weight['curb_weight_lbs'],
            cg_height=weight['cg_height_inches'],
            track_width=(basic['front_track_inches'] + basic['rear_track_inches']) / 2
        )

        # Estimate roll stiffness
        front_k_roll = estimate_roll_stiffness(
            spring_rate=spring['front_spring_rate_lbs_in'],
            motion_ratio=0.7,
            track_width=basic['front_track_inches']
        )

        rear_k_roll = estimate_roll_stiffness(
            spring_rate=spring['rear_spring_rate_lbs_in'],
            motion_ratio=0.65,
            track_width=basic['rear_track_inches']
        )

        # Distribute load transfer
        lt_distribution = distribute_load_transfer(
            total_lateral_transfer=total_lt,
            total_weight=weight['curb_weight_lbs'],
            wheelbase=basic['wheelbase_inches'],
            cg_from_front_axle=weight['cg_from_front_axle_inches'],
            front_track=basic['front_track_inches'],
            rear_track=basic['rear_track_inches'],
            front_roll_stiffness=front_k_roll,
            rear_roll_stiffness=rear_k_roll
        )

        # Analyze handling balance
        balance = analyze_handling_balance(
            front_lateral_transfer_pct=lt_distribution.front_percentage,
            rear_lateral_transfer_pct=lt_distribution.rear_percentage,
            front_weight_pct=weight['front_weight_percent']
        )

        return {
            'lateral_g': lateral_g,
            'total_transfer_lbs': total_lt,
            'front_transfer_lbs': lt_distribution.front_transfer,
            'rear_transfer_lbs': lt_distribution.rear_transfer,
            'front_percentage': lt_distribution.front_percentage,
            'rear_percentage': lt_distribution.rear_percentage,
            'handling_balance': balance
        }

    def generate_summary(self) -> Dict:
        """Generate summary of all analysis results"""

        if 'front' not in self.results:
            self.analyze_front_suspension()

        # Get key metrics
        front = self.results['front']
        load_transfer = self.analyze_load_transfer(1.0)

        summary = {
            'vehicle': f"{self.vehicle['vehicle_info']['year']} {self.vehicle['vehicle_info']['make']} {self.vehicle['vehicle_info']['model']}",
            'front_suspension_type': self.vehicle['front_suspension']['type'],

            'roll_center': {
                'height_inches': front['roll_center']['height'],
                'assessment': front['roll_center']['evaluation']['assessment']
            },

            'camber_gain': {
                'deg_per_inch': front['camber_gain']['gain_deg_per_inch'],
                'assessment': front['camber_gain']['assessment']
            },

            'anti_dive': {
                'percent': front['anti_dive_percent'],
                'assessment': self._assess_anti_dive(front['anti_dive_percent'])
            },

            'handling_balance': load_transfer['handling_balance']['balance'],

            'key_recommendations': self._generate_recommendations()
        }

        return summary

    def _assess_anti_dive(self, percent: float) -> str:
        """Assess anti-dive percentage"""
        if 30 <= percent <= 60:
            return "Optimal for street use"
        elif 20 <= percent <= 70:
            return "Acceptable"
        elif percent < 20:
            return "Low - expect brake dive"
        else:
            return "High - may cause harsh braking feel"

    def _generate_recommendations(self) -> List[str]:
        """Generate top recommendations based on analysis"""
        recommendations = []

        front = self.results.get('front', {})

        # Roll center recommendations
        rc_eval = front.get('roll_center', {}).get('evaluation', {})
        if rc_eval.get('assessment') not in ['Optimal', 'Acceptable']:
            recommendations.append(f"Consider roll center adjustment - current assessment: {rc_eval.get('assessment')}")

        # Camber gain recommendations
        camber = front.get('camber_gain', {})
        if "Low" in camber.get('assessment', ''):
            recommendations.append("Add camber plates to improve camber gain")

        # Load transfer
        lt = self.analyze_load_transfer(1.0)
        if "Understeer" in lt['handling_balance']['balance']:
            recommendations.append("Reduce front roll stiffness to reduce understeer")
        elif "Oversteer" in lt['handling_balance']['balance']:
            recommendations.append("Increase front roll stiffness to reduce oversteer")

        if not recommendations:
            recommendations.append("Current geometry is well-optimized")

        return recommendations


if __name__ == "__main__":
    print("MacPherson Strut Analyzer\n")

    # Load vehicle data
    import os
    config_path = os.path.join(os.path.dirname(__file__), '../../config/vehicle_templates/1992_celica_gt.json')

    try:
        with open(config_path, 'r') as f:
            vehicle_data = json.load(f)

        # Run analysis
        analyzer = MacPhersonStrutAnalyzer(vehicle_data)
        front_results = analyzer.analyze_front_suspension()

        print(f"Roll Center Height: {front_results['roll_center']['height']:.2f}\"")
        print(f"Camber Gain: {front_results['camber_gain']['gain_deg_per_inch']:.3f} deg/inch")
        print(f"Anti-Dive: {front_results['anti_dive_percent']:.1f}%")

        # Load transfer analysis
        lt = analyzer.analyze_load_transfer(1.0)
        print(f"\nAt 1.0g lateral:")
        print(f"  Front LT: {lt['front_transfer_lbs']:.0f} lbs ({lt['front_percentage']:.1f}%)")
        print(f"  Handling: {lt['handling_balance']['balance']}")

    except FileNotFoundError:
        print(f"Vehicle config not found at: {config_path}")
        print("Run from project root or provide config path")
