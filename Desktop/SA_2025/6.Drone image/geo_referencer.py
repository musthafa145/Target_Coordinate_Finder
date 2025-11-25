"""
Geo-Referencing Module
Converts detected objects in drone images to real-world GPS coordinates.
Integrates image processing with coordinate transformations.
"""

import numpy as np
from typing import List, Dict, Tuple
from dataclasses import dataclass
from coordinate_transform import CoordinateTransform


@dataclass
class DroneState:
    """Represents drone position and orientation at image capture."""
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    yaw_deg: float = 0.0  # Rotation around up axis
    pitch_deg: float = -90.0  # Camera pointing down
    roll_deg: float = 0.0


@dataclass
class DetectedObject:
    """Represents a detected object with real-world position."""
    class_name: str
    confidence: float
    pixel_box: List[int]  # [x1, y1, x2, y2]
    pixel_center: Tuple[float, float]
    gps_latitude: float
    gps_longitude: float
    altitude_m: float
    enu_offset: np.ndarray  # [east, north, up]


class GeoReferencer:
    """
    Converts pixel detections to GPS coordinates using drone state and camera calibration.
    """

    def __init__(self, ref_latitude_deg: float, ref_longitude_deg: float, 
                 ref_altitude_m: float = 0.0):
        """
        Initialize geo-referencer with reference point.
        
        Args:
            ref_latitude_deg, ref_longitude_deg, ref_altitude_m: Reference GPS location
        """
        self.coord_transform = CoordinateTransform(
            ref_latitude_deg, ref_longitude_deg, ref_altitude_m
        )

    def detect_to_gps(self, detection: Dict, drone_state: DroneState,
                     image_width_px: int, image_height_px: int,
                     gsd: float) -> DetectedObject:
        """
        Convert image detection to GPS coordinates.
        
        Args:
            detection: Single detection dict with 'boxes', 'classes', etc.
            drone_state: Drone position/orientation at image capture
            image_width_px, image_height_px: Image dimensions
            gsd: Ground Sample Distance (m/pixel)
            
        Returns:
            DetectedObject with real-world GPS position
        """
        # Get center pixel
        box = detection['boxes']
        center_x, center_y = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        
        # Calculate offset from drone nadir point (in meters)
        east, north = self._pixel_to_enu_nadir(
            center_x, center_y, image_width_px, image_height_px, gsd
        )
        
        # Apply drone yaw rotation
        east_rot, north_rot = self._rotate_enu(east, north, drone_state.yaw_deg)
        
        # Get GPS of object
        gps_lat, gps_lon = self.coord_transform.enu_to_gps(
            np.array([east_rot, north_rot, 0])
        )
        
        # ENU coordinates relative to reference
        global_enu = self.coord_transform.gps_to_enu(gps_lat, gps_lon, drone_state.altitude_m)
        
        return DetectedObject(
            class_name=detection['class_names'],
            confidence=detection['confidences'],
            pixel_box=box,
            pixel_center=(center_x, center_y),
            gps_latitude=gps_lat,
            gps_longitude=gps_lon,
            altitude_m=drone_state.altitude_m,
            enu_offset=global_enu
        )

    def batch_detect_to_gps(self, detections: Dict, drone_state: DroneState,
                           image_width_px: int, image_height_px: int,
                           gsd: float) -> List[DetectedObject]:
        """
        Convert multiple detections to GPS.
        
        Args:
            detections: Dict with 'boxes', 'classes', 'confidences', 'class_names'
            drone_state: Drone state
            image_width_px, image_height_px: Image dimensions
            gsd: Ground Sample Distance
            
        Returns:
            List of DetectedObject with GPS coordinates
        """
        geo_objects = []
        
        for i in range(len(detections['boxes'])):
            single_detection = {
                'boxes': detections['boxes'][i],
                'classes': detections['classes'][i],
                'confidences': detections['confidences'][i],
                'class_names': detections['class_names'][i]
            }
            
            geo_obj = self.detect_to_gps(
                single_detection, drone_state,
                image_width_px, image_height_px, gsd
            )
            geo_objects.append(geo_obj)
        
        return geo_objects

    def _pixel_to_enu_nadir(self, center_x_px: float, center_y_px: float,
                            image_width_px: int, image_height_px: int,
                            gsd: float) -> Tuple[float, float]:
        """
        Convert pixel coordinates to ENU offset (nadir view).
        
        Args:
            center_x_px, center_y_px: Pixel coordinates
            image_width_px, image_height_px: Image dimensions
            gsd: Ground Sample Distance
            
        Returns:
            tuple: (east, north) offset in meters
        """
        pixel_x_offset = center_x_px - image_width_px / 2.0
        pixel_y_offset = center_y_px - image_height_px / 2.0
        
        east = pixel_x_offset * gsd
        north = -pixel_y_offset * gsd
        
        return east, north

    def _rotate_enu(self, east: float, north: float, yaw_deg: float) -> Tuple[float, float]:
        """
        Rotate ENU coordinates by drone yaw.
        
        Args:
            east, north: ENU offsets
            yaw_deg: Yaw angle in degrees
            
        Returns:
            tuple: (east_rotated, north_rotated)
        """
        yaw_rad = np.radians(yaw_deg)
        east_rot = east * np.cos(yaw_rad) - north * np.sin(yaw_rad)
        north_rot = east * np.sin(yaw_rad) + north * np.cos(yaw_rad)
        return east_rot, north_rot

    def export_detections_csv(self, geo_objects: List[DetectedObject], 
                             output_file: str):
        """
        Export detected objects to CSV.
        
        Args:
            geo_objects: List of DetectedObject
            output_file: Output CSV file path
        """
        import csv
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Object', 'Confidence', 'Latitude', 'Longitude', 
                           'Altitude (m)', 'East (m)', 'North (m)', 'Up (m)'])
            
            for obj in geo_objects:
                writer.writerow([
                    obj.class_name,
                    f"{obj.confidence:.2f}",
                    f"{obj.gps_latitude:.6f}",
                    f"{obj.gps_longitude:.6f}",
                    f"{obj.altitude_m:.1f}",
                    f"{obj.enu_offset[0]:.1f}",
                    f"{obj.enu_offset[1]:.1f}",
                    f"{obj.enu_offset[2]:.1f}"
                ])

    def export_detections_json(self, geo_objects: List[DetectedObject], 
                              output_file: str):
        """
        Export detected objects to JSON.
        
        Args:
            geo_objects: List of DetectedObject
            output_file: Output JSON file path
        """
        import json
        
        data = {
            'detections': [
                {
                    'class': obj.class_name,
                    'confidence': obj.confidence,
                    'gps': {
                        'latitude': obj.gps_latitude,
                        'longitude': obj.gps_longitude,
                        'altitude': obj.altitude_m
                    },
                    'enu': {
                        'east': float(obj.enu_offset[0]),
                        'north': float(obj.enu_offset[1]),
                        'up': float(obj.enu_offset[2])
                    },
                    'pixel_center': obj.pixel_center
                }
                for obj in geo_objects
            ]
        }
        
        with open(output_file, 'w') as f:
            json.dump(data, f, indent=2)
