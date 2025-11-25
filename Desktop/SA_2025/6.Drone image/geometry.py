"""
Geometry Engine Module
Handles 3D geometry operations: ray-ground intersections, rotations, coordinate frames.
Uses Z-Y-X Euler angle convention for drone and gimbal attitude.
"""

import numpy as np


class GeometryEngine:
    """
    Computes geometric operations for the pixel-to-GPS pipeline.
    - Builds rotation matrices from flight and gimbal attitudes
    - Transforms rays through camera, drone, and world frames
    - Finds ray-ground plane intersections
    """

    def __init__(self, drone_altitude_m: float = 100.0,
                 gimbal_pitch_deg: float = 0.0,
                 gimbal_roll_deg: float = 0.0,
                 gimbal_yaw_deg: float = 0.0,
                 flight_yaw_deg: float = 0.0,
                 flight_pitch_deg: float = 0.0,
                 flight_roll_deg: float = 0.0):
        """
        Initialize geometry engine with drone and gimbal orientation.
        
        Args:
            drone_altitude_m: Drone height above ground (meters)
            gimbal_pitch_deg, gimbal_roll_deg, gimbal_yaw_deg: Gimbal Euler angles
            flight_yaw_deg, flight_pitch_deg, flight_roll_deg: Flight Euler angles
        """
        self.drone_altitude_m = drone_altitude_m
        self.gimbal_pitch_deg = gimbal_pitch_deg
        self.gimbal_roll_deg = gimbal_roll_deg
        self.gimbal_yaw_deg = gimbal_yaw_deg
        self.flight_yaw_deg = flight_yaw_deg
        self.flight_pitch_deg = flight_pitch_deg
        self.flight_roll_deg = flight_roll_deg

        # Pre-compute rotation matrices
        self.R_gimbal = self._euler_to_rotation_matrix(gimbal_yaw_deg, gimbal_pitch_deg, gimbal_roll_deg)
        self.R_flight = self._euler_to_rotation_matrix(flight_yaw_deg, flight_pitch_deg, flight_roll_deg)
        self.R_camera_to_world = self.R_flight @ self.R_gimbal

    @staticmethod
    def _euler_to_rotation_matrix(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
        """
        Build rotation matrix from Z-Y-X Euler angles.
        
        Convention: Yaw (Z) → Pitch (Y) → Roll (X)
        
        Args:
            yaw_deg, pitch_deg, roll_deg: Angles in degrees
            
        Returns:
            ndarray: 3x3 rotation matrix
        """
        yaw = np.radians(yaw_deg)
        pitch = np.radians(pitch_deg)
        roll = np.radians(roll_deg)

        # Z rotation (yaw)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Y rotation (pitch)
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        # X rotation (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        return Rz @ Ry @ Rx

    def transform_camera_ray_to_world(self, camera_ray: np.ndarray) -> np.ndarray:
        """
        Transform camera ray to world frame (ENU).
        
        Args:
            camera_ray: Ray direction in camera frame
            
        Returns:
            ndarray: Ray direction in world frame
        """
        # If camera is approximately pointing straight down (common nadir case),
        # return a downward world ray rotated by flight yaw so the intersection
        # math remains stable (avoids near-zero z components).
        try:
            pitch = self.gimbal_pitch_deg
        except Exception:
            pitch = 0.0

        # Detect near-nadir camera ray (small lateral components) and pitch ~ -90°
        if abs(pitch + 90.0) < 1.0 and abs(camera_ray[0]) < 1e-3 and abs(camera_ray[1]) < 1e-3:
            # Downward vector in body/world before yaw
            down = np.array([0.0, 0.0, -1.0])
            # Apply flight yaw so 'down' is oriented with drone yaw
            yaw_rad = np.radians(self.flight_yaw_deg)
            Rz = np.array([
                [np.cos(yaw_rad), -np.sin(yaw_rad), 0.0],
                [np.sin(yaw_rad), np.cos(yaw_rad), 0.0],
                [0.0, 0.0, 1.0]
            ])
            return Rz @ down

        return self.R_camera_to_world @ camera_ray

    def ray_ground_intersection(self, world_ray: np.ndarray, drone_position_enu: np.ndarray,
                               ground_z: float = 0.0) -> np.ndarray:
        """
        Find intersection of ray with ground plane (z = ground_z).
        
        Ray: P(t) = ray_origin + t * ray_direction
        Ground: z = ground_z
        
        Args:
            world_ray: Ray direction in world frame (normalized)
            drone_position_enu: Drone position in ENU coordinates
            ground_z: Ground plane z-coordinate (default: 0, ground level)
            
        Returns:
            ndarray: Intersection point in ENU, or None if no intersection
        """
        # Ray starts at drone altitude (ray_origin at z = altitude)
        ray_origin = drone_position_enu.copy()
        ray_origin[2] = self.drone_altitude_m  # Set z to drone altitude

        # Solve: ray_origin[2] + t * ray_direction[2] = ground_z
        denom = world_ray[2]

        if abs(denom) < 1e-6:
            return None  # Ray parallel to ground

        t = (ground_z - ray_origin[2]) / denom

        if t < 0:
            return None  # Intersection behind ray origin

        intersection = ray_origin + t * world_ray
        return intersection
