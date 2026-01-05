"""
Track and Lap Analysis Module.

This module provides GPS-based track analysis including:
- Track mapping with parameter overlay
- Lap timing and sector analysis
- Racing line comparison
- Braking zone analysis
- Corner speed analysis
"""

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import interpolate

logger = logging.getLogger(__name__)


@dataclass
class TrackPoint:
    """Single point on the track."""
    lat: float
    lon: float
    elapsed_time: float = 0.0
    speed_mph: float = 0.0
    lateral_g: float = 0.0
    longitudinal_g: float = 0.0
    throttle_pct: float = 0.0
    brake_pressure: float = 0.0
    distance_m: float = 0.0


@dataclass
class LapData:
    """Data for a single lap."""
    lap_number: int
    lap_time_sec: float
    start_time: float
    end_time: float
    points: List[TrackPoint] = field(default_factory=list)
    sectors: Dict[str, float] = field(default_factory=dict)
    max_speed_mph: float = 0.0
    avg_speed_mph: float = 0.0
    max_lateral_g: float = 0.0
    max_braking_g: float = 0.0


@dataclass
class CornerData:
    """Data for a single corner."""
    corner_id: int
    name: str = ""
    entry_speed_mph: float = 0.0
    apex_speed_mph: float = 0.0
    exit_speed_mph: float = 0.0
    min_speed_mph: float = 0.0
    max_lateral_g: float = 0.0
    braking_distance_m: float = 0.0
    corner_radius_m: float = 0.0
    direction: str = ""  # "left" or "right"


@dataclass
class SectorData:
    """Data for a track sector."""
    sector_name: str
    sector_time_sec: float
    distance_m: float
    avg_speed_mph: float
    max_speed_mph: float
    min_speed_mph: float


class TrackAnalyzer:
    """
    GPS Track and Lap Analyzer.

    Analyzes track data including lap times, sector times,
    racing line, and corner speeds.
    """

    EARTH_RADIUS_M = 6371000

    def __init__(
        self,
        start_finish_lat: float,
        start_finish_lon: float,
        detection_radius_m: float = 25.0,
        min_lap_time_sec: float = 60.0,
        max_lap_time_sec: float = 300.0,
        sector_definitions: Optional[List[Dict]] = None
    ):
        """
        Initialize track analyzer.

        Args:
            start_finish_lat: Start/finish line latitude
            start_finish_lon: Start/finish line longitude
            detection_radius_m: Detection radius for line crossing
            min_lap_time_sec: Minimum valid lap time
            max_lap_time_sec: Maximum valid lap time
            sector_definitions: List of sector boundary definitions
        """
        self.sf_lat = start_finish_lat
        self.sf_lon = start_finish_lon
        self.detection_radius = detection_radius_m
        self.min_lap_time = min_lap_time_sec
        self.max_lap_time = max_lap_time_sec
        self.sectors = sector_definitions or []

    @staticmethod
    def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate distance between two GPS points using Haversine formula.

        Args:
            lat1, lon1: First point coordinates
            lat2, lon2: Second point coordinates

        Returns:
            Distance in meters
        """
        R = 6371000  # Earth's radius in meters

        lat1_rad = np.radians(lat1)
        lat2_rad = np.radians(lat2)
        dlat = np.radians(lat2 - lat1)
        dlon = np.radians(lon2 - lon1)

        a = (np.sin(dlat/2)**2 +
             np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon/2)**2)
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))

        return R * c

    def detect_lap_crossings(
        self,
        timestamps: np.ndarray,
        latitudes: np.ndarray,
        longitudes: np.ndarray
    ) -> List[int]:
        """
        Detect start/finish line crossings.

        Args:
            timestamps: Time array
            latitudes: GPS latitude array
            longitudes: GPS longitude array

        Returns:
            List of crossing indices
        """
        crossings = []
        in_zone = False
        last_crossing_time = 0.0

        for i in range(len(latitudes)):
            distance = self.haversine_distance(
                latitudes[i], longitudes[i],
                self.sf_lat, self.sf_lon
            )

            currently_in_zone = distance < self.detection_radius

            # Detect entering the zone
            if currently_in_zone and not in_zone:
                time_since_last = timestamps[i] - last_crossing_time

                # Check if this is a valid new lap
                if time_since_last >= self.min_lap_time:
                    crossings.append(i)
                    last_crossing_time = timestamps[i]

            in_zone = currently_in_zone

        return crossings

    def calculate_cumulative_distance(
        self,
        latitudes: np.ndarray,
        longitudes: np.ndarray
    ) -> np.ndarray:
        """
        Calculate cumulative distance traveled.

        Args:
            latitudes: GPS latitude array
            longitudes: GPS longitude array

        Returns:
            Cumulative distance array in meters
        """
        distances = np.zeros(len(latitudes))

        for i in range(1, len(latitudes)):
            distances[i] = distances[i-1] + self.haversine_distance(
                latitudes[i-1], longitudes[i-1],
                latitudes[i], longitudes[i]
            )

        return distances

    def extract_laps(
        self,
        timestamps: np.ndarray,
        latitudes: np.ndarray,
        longitudes: np.ndarray,
        speeds_mph: np.ndarray,
        lateral_g: np.ndarray,
        longitudinal_g: np.ndarray,
        throttle_pct: Optional[np.ndarray] = None,
        brake_pressure: Optional[np.ndarray] = None
    ) -> List[LapData]:
        """
        Extract individual laps from session data.

        Args:
            timestamps: Time array
            latitudes: GPS latitude array
            longitudes: GPS longitude array
            speeds_mph: Speed array
            lateral_g: Lateral acceleration
            longitudinal_g: Longitudinal acceleration
            throttle_pct: Optional throttle position
            brake_pressure: Optional brake pressure

        Returns:
            List of LapData objects
        """
        crossings = self.detect_lap_crossings(timestamps, latitudes, longitudes)
        laps = []

        if throttle_pct is None:
            throttle_pct = np.zeros_like(timestamps)
        if brake_pressure is None:
            brake_pressure = np.zeros_like(timestamps)

        for lap_num, (start_idx, end_idx) in enumerate(zip(crossings[:-1], crossings[1:])):
            lap_time = timestamps[end_idx] - timestamps[start_idx]

            # Skip invalid lap times
            if lap_time < self.min_lap_time or lap_time > self.max_lap_time:
                continue

            # Create track points
            points = []
            distances = self.calculate_cumulative_distance(
                latitudes[start_idx:end_idx],
                longitudes[start_idx:end_idx]
            )

            for i in range(start_idx, end_idx):
                rel_i = i - start_idx
                points.append(TrackPoint(
                    lat=latitudes[i],
                    lon=longitudes[i],
                    elapsed_time=timestamps[i] - timestamps[start_idx],
                    speed_mph=speeds_mph[i],
                    lateral_g=lateral_g[i],
                    longitudinal_g=longitudinal_g[i],
                    throttle_pct=throttle_pct[i],
                    brake_pressure=brake_pressure[i],
                    distance_m=distances[rel_i] if rel_i < len(distances) else 0
                ))

            lap = LapData(
                lap_number=lap_num + 1,
                lap_time_sec=lap_time,
                start_time=timestamps[start_idx],
                end_time=timestamps[end_idx],
                points=points,
                max_speed_mph=float(np.max(speeds_mph[start_idx:end_idx])),
                avg_speed_mph=float(np.mean(speeds_mph[start_idx:end_idx])),
                max_lateral_g=float(np.max(np.abs(lateral_g[start_idx:end_idx]))),
                max_braking_g=float(np.min(longitudinal_g[start_idx:end_idx]))
            )

            laps.append(lap)

        return laps

    def identify_corners(
        self,
        lap: LapData,
        lateral_g_threshold: float = 0.3,
        min_duration_sec: float = 1.0
    ) -> List[CornerData]:
        """
        Identify corners within a lap.

        Args:
            lap: Lap data
            lateral_g_threshold: Minimum lateral g for corner detection
            min_duration_sec: Minimum corner duration

        Returns:
            List of CornerData objects
        """
        corners = []
        points = lap.points

        if len(points) < 10:
            return corners

        # Extract arrays
        times = np.array([p.elapsed_time for p in points])
        lat_g = np.array([p.lateral_g for p in points])
        speeds = np.array([p.speed_mph for p in points])

        # Find cornering sections
        in_corner = np.abs(lat_g) > lateral_g_threshold
        transitions = np.diff(in_corner.astype(int))
        starts = np.where(transitions == 1)[0]
        ends = np.where(transitions == -1)[0]

        # Match starts and ends
        corner_id = 0
        for start in starts:
            possible_ends = ends[ends > start]
            if len(possible_ends) == 0:
                continue

            end = possible_ends[0]
            duration = times[end] - times[start]

            if duration < min_duration_sec:
                continue

            # Analyze corner
            corner_lat_g = lat_g[start:end]
            corner_speeds = speeds[start:end]

            # Determine direction
            avg_lat_g = np.mean(corner_lat_g)
            direction = "right" if avg_lat_g > 0 else "left"

            # Entry/apex/exit speeds
            n = len(corner_speeds)
            entry_speed = np.mean(corner_speeds[:max(1, n//5)])
            exit_speed = np.mean(corner_speeds[max(0, n - n//5):])
            min_speed = np.min(corner_speeds)
            apex_idx = np.argmax(np.abs(corner_lat_g))
            apex_speed = corner_speeds[apex_idx]

            # Estimate corner radius from speed and lateral g
            # R = v² / (g * lateral_g)
            max_g = np.max(np.abs(corner_lat_g))
            if max_g > 0:
                v_m_s = apex_speed * 0.44704
                radius = v_m_s ** 2 / (9.81 * max_g)
            else:
                radius = 0

            corner_id += 1
            corners.append(CornerData(
                corner_id=corner_id,
                name=f"Corner {corner_id}",
                entry_speed_mph=entry_speed,
                apex_speed_mph=apex_speed,
                exit_speed_mph=exit_speed,
                min_speed_mph=min_speed,
                max_lateral_g=max_g,
                corner_radius_m=radius,
                direction=direction
            ))

        return corners

    def analyze_braking_zones(
        self,
        lap: LapData,
        braking_g_threshold: float = -0.3
    ) -> List[Dict]:
        """
        Analyze braking zones.

        Args:
            lap: Lap data
            braking_g_threshold: Threshold for braking detection (negative)

        Returns:
            List of braking zone data
        """
        zones = []
        points = lap.points

        if len(points) < 10:
            return zones

        lon_g = np.array([p.longitudinal_g for p in points])
        speeds = np.array([p.speed_mph for p in points])
        distances = np.array([p.distance_m for p in points])

        # Find braking sections
        braking = lon_g < braking_g_threshold
        transitions = np.diff(braking.astype(int))
        starts = np.where(transitions == 1)[0]
        ends = np.where(transitions == -1)[0]

        for i, start in enumerate(starts):
            possible_ends = ends[ends > start]
            if len(possible_ends) == 0:
                continue

            end = possible_ends[0]

            zone = {
                'zone_id': i + 1,
                'start_distance_m': distances[start],
                'end_distance_m': distances[end],
                'braking_distance_m': distances[end] - distances[start],
                'entry_speed_mph': speeds[start],
                'exit_speed_mph': speeds[end],
                'speed_reduction_mph': speeds[start] - speeds[end],
                'max_braking_g': float(np.min(lon_g[start:end])),
                'avg_braking_g': float(np.mean(lon_g[start:end]))
            }

            zones.append(zone)

        return zones

    def compare_laps(
        self,
        lap1: LapData,
        lap2: LapData,
        distance_step_m: float = 10.0
    ) -> Dict:
        """
        Compare two laps.

        Args:
            lap1: First lap (typically reference/faster)
            lap2: Second lap
            distance_step_m: Interpolation distance step

        Returns:
            Comparison data dictionary
        """
        # Create distance-based comparison
        if not lap1.points or not lap2.points:
            return {}

        # Get arrays
        d1 = np.array([p.distance_m for p in lap1.points])
        s1 = np.array([p.speed_mph for p in lap1.points])
        t1 = np.array([p.elapsed_time for p in lap1.points])

        d2 = np.array([p.distance_m for p in lap2.points])
        s2 = np.array([p.speed_mph for p in lap2.points])
        t2 = np.array([p.elapsed_time for p in lap2.points])

        # Common distance range
        max_dist = min(d1[-1], d2[-1])
        distances = np.arange(0, max_dist, distance_step_m)

        # Interpolate to common distance
        s1_interp = np.interp(distances, d1, s1)
        s2_interp = np.interp(distances, d2, s2)
        t1_interp = np.interp(distances, d1, t1)
        t2_interp = np.interp(distances, d2, t2)

        # Calculate deltas
        speed_delta = s1_interp - s2_interp  # Positive = lap1 faster
        time_delta = t2_interp - t1_interp  # Positive = lap1 ahead

        return {
            'distances': distances,
            'speed_delta': speed_delta,
            'time_delta': time_delta,
            'cumulative_time_delta': time_delta,  # Already cumulative from interpolation
            'lap1_speed': s1_interp,
            'lap2_speed': s2_interp,
            'total_time_diff': lap2.lap_time_sec - lap1.lap_time_sec,
            'avg_speed_diff': lap1.avg_speed_mph - lap2.avg_speed_mph,
            'max_speed_diff': lap1.max_speed_mph - lap2.max_speed_mph
        }

    def generate_track_map(
        self,
        lap: LapData,
        parameter: str = 'speed_mph'
    ) -> Dict:
        """
        Generate track map data with color-coded parameter.

        Args:
            lap: Lap data
            parameter: Parameter to color-code ('speed_mph', 'lateral_g', etc.)

        Returns:
            Dictionary with track map data
        """
        points = lap.points

        lats = np.array([p.lat for p in points])
        lons = np.array([p.lon for p in points])

        if parameter == 'speed_mph':
            values = np.array([p.speed_mph for p in points])
        elif parameter == 'lateral_g':
            values = np.array([p.lateral_g for p in points])
        elif parameter == 'longitudinal_g':
            values = np.array([p.longitudinal_g for p in points])
        elif parameter == 'throttle_pct':
            values = np.array([p.throttle_pct for p in points])
        else:
            values = np.array([p.speed_mph for p in points])

        return {
            'latitudes': lats,
            'longitudes': lons,
            'values': values,
            'parameter': parameter,
            'min_value': float(np.min(values)),
            'max_value': float(np.max(values)),
            'lap_number': lap.lap_number,
            'lap_time': lap.lap_time_sec
        }

    def analyze_session(
        self,
        timestamps: np.ndarray,
        latitudes: np.ndarray,
        longitudes: np.ndarray,
        speeds_mph: np.ndarray,
        lateral_g: np.ndarray,
        longitudinal_g: np.ndarray,
        **kwargs
    ) -> Dict:
        """
        Perform complete session analysis.

        Args:
            timestamps: Time array
            latitudes: GPS latitude array
            longitudes: GPS longitude array
            speeds_mph: Speed array
            lateral_g: Lateral acceleration
            longitudinal_g: Longitudinal acceleration
            **kwargs: Additional data arrays

        Returns:
            Complete analysis results
        """
        # Extract laps
        laps = self.extract_laps(
            timestamps, latitudes, longitudes,
            speeds_mph, lateral_g, longitudinal_g,
            throttle_pct=kwargs.get('throttle_pct'),
            brake_pressure=kwargs.get('brake_pressure')
        )

        results = {
            'total_laps': len(laps),
            'laps': []
        }

        if not laps:
            return results

        # Find best lap
        lap_times = [lap.lap_time_sec for lap in laps]
        best_lap_idx = np.argmin(lap_times)

        results['best_lap_number'] = laps[best_lap_idx].lap_number
        results['best_lap_time'] = laps[best_lap_idx].lap_time_sec
        results['avg_lap_time'] = float(np.mean(lap_times))
        results['lap_time_std'] = float(np.std(lap_times))

        # Analyze each lap
        for lap in laps:
            lap_results = {
                'lap_number': lap.lap_number,
                'lap_time': lap.lap_time_sec,
                'max_speed': lap.max_speed_mph,
                'avg_speed': lap.avg_speed_mph,
                'max_lateral_g': lap.max_lateral_g,
                'max_braking_g': lap.max_braking_g,
                'corners': [],
                'braking_zones': []
            }

            # Corners
            corners = self.identify_corners(lap)
            for corner in corners:
                lap_results['corners'].append({
                    'id': corner.corner_id,
                    'direction': corner.direction,
                    'apex_speed': corner.apex_speed_mph,
                    'max_g': corner.max_lateral_g
                })

            # Braking zones
            braking = self.analyze_braking_zones(lap)
            lap_results['braking_zones'] = braking

            results['laps'].append(lap_results)

        # Compare best and worst
        if len(laps) > 1:
            worst_lap_idx = np.argmax(lap_times)
            results['comparison'] = self.compare_laps(
                laps[best_lap_idx],
                laps[worst_lap_idx]
            )

        return results
