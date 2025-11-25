"""
Coordinate Transform Module
Handles conversions between GPS, ENU (East-North-Up), and local coordinate systems.
Uses equirectangular approximation for small distances (<100 km).
"""

import numpy as np


class CoordinateTransform:
    """
    Manages coordinate system conversions: GPS ↔ ENU (East-North-Up local frame).
    Assumes flat-earth approximation (valid for distances < 100 km).
    """

    EARTH_RADIUS_M = 6371000.0  # WGS84 mean radius

    def __init__(self, ref_latitude_deg: float, ref_longitude_deg: float, ref_altitude_m: float = 0.0):
        """
        Initialize with reference GPS point (local origin).
        
        Args:
            ref_latitude_deg: Reference latitude in degrees
            ref_longitude_deg: Reference longitude in degrees
            ref_altitude_m: Reference altitude in meters (default: 0, sea level)
        """
        self.ref_latitude_deg = ref_latitude_deg
        self.ref_longitude_deg = ref_longitude_deg
        self.ref_altitude_m = ref_altitude_m
        self.ref_latitude_rad = np.radians(ref_latitude_deg)

    def drone_to_enu(self, drone_lat_deg: float, drone_lon_deg: float, drone_alt_m: float) -> np.ndarray:
        """
        Convert drone GPS position to ENU relative to reference.
        
        Args:
            drone_lat_deg, drone_lon_deg, drone_alt_m: Drone GPS coordinates
            
        Returns:
            ndarray: [east, north, up] ENU position in meters
        """
        # Latitude/longitude differences
        dlat_rad = np.radians(drone_lat_deg - self.ref_latitude_deg)
        dlon_rad = np.radians(drone_lon_deg - self.ref_longitude_deg)

        # ENU coordinates (equirectangular approximation)
        east = self.EARTH_RADIUS_M * dlon_rad * np.cos(self.ref_latitude_rad)
        north = self.EARTH_RADIUS_M * dlat_rad
        up = drone_alt_m - self.ref_altitude_m

        return np.array([east, north, up])

    def enu_to_gps(self, enu_offset: np.ndarray) -> tuple:
        """
        Convert ENU offset to GPS coordinates.
        
        Args:
            enu_offset: [east, north, up] in meters
            
        Returns:
            tuple: (latitude_deg, longitude_deg)
        """
        east, north, up = enu_offset

        # Reverse equirectangular transformation
        dlat_rad = north / self.EARTH_RADIUS_M
        dlon_rad = east / (self.EARTH_RADIUS_M * np.cos(self.ref_latitude_rad))

        latitude_deg = self.ref_latitude_deg + np.degrees(dlat_rad)
        longitude_deg = self.ref_longitude_deg + np.degrees(dlon_rad)

        return latitude_deg, longitude_deg

    def gps_to_enu(self, latitude_deg: float, longitude_deg: float, altitude_m: float = 0.0) -> np.ndarray:
        """
        Convert absolute GPS to ENU relative to reference.
        
        Args:
            latitude_deg, longitude_deg, altitude_m: GPS coordinates
            
        Returns:
            ndarray: [east, north, up] ENU offset
        """
        dlat_rad = np.radians(latitude_deg - self.ref_latitude_deg)
        dlon_rad = np.radians(longitude_deg - self.ref_longitude_deg)

        east = self.EARTH_RADIUS_M * dlon_rad * np.cos(self.ref_latitude_rad)
        north = self.EARTH_RADIUS_M * dlat_rad
        up = altitude_m - self.ref_altitude_m

        return np.array([east, north, up])

    def enu_to_global_enu(self, local_enu: np.ndarray, drone_lat: float, drone_lon: float, drone_alt: float) -> np.ndarray:
        """
        Transform local ENU (drone-centered) to global ENU (reference-centered).
        
        Args:
            local_enu: Offset in drone-centered ENU
            drone_lat, drone_lon, drone_alt: Drone GPS position
            
        Returns:
            ndarray: Global ENU coordinates relative to reference
        """
        drone_enu = self.drone_to_enu(drone_lat, drone_lon, drone_alt)
        return drone_enu + local_enu
