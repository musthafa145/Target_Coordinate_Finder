"""
Camera Model Module
Encapsulates camera intrinsic parameters and pixel-to-ray conversions.
Handles focal length, sensor dimensions, and normalized camera coordinates.
"""

import numpy as np


class CameraModel:
    """
    Represents a camera with intrinsic parameters.
    Converts pixel coordinates to normalized camera rays.
    """

    # Default DJI Matrice 200 Series sensor specs
    DEFAULT_FOCAL_LENGTH_MM = 3.6
    DEFAULT_SENSOR_WIDTH_MM = 6.17
    DEFAULT_SENSOR_HEIGHT_MM = 4.63

    def __init__(self, metadata: dict, sensor_width_mm: float = None, sensor_height_mm: float = None):
        """
        Initialize camera model from metadata.
        
        Args:
            metadata: Dict with 'image_width', 'image_height', 'focal_length_mm'
            sensor_width_mm: Physical sensor width (default: DJI FC300S)
            sensor_height_mm: Physical sensor height (default: DJI FC300S)
        """
        self.image_width = metadata.get('image_width', 4000)
        self.image_height = metadata.get('image_height', 3000)
        self.focal_length_mm = metadata.get('focal_length_mm', self.DEFAULT_FOCAL_LENGTH_MM)

        # Use provided or default sensor dimensions
        self.sensor_width_mm = sensor_width_mm or self.DEFAULT_SENSOR_WIDTH_MM
        self.sensor_height_mm = sensor_height_mm or self.DEFAULT_SENSOR_HEIGHT_MM

        # Compute intrinsic matrix K
        self._compute_intrinsics()

    def _compute_intrinsics(self):
        """Compute focal length in pixels and principal point."""
        # Pixel dimensions in mm
        pixel_size_x = self.sensor_width_mm / self.image_width
        pixel_size_y = self.sensor_height_mm / self.image_height

        # Focal length in pixels
        self.fx = self.focal_length_mm / pixel_size_x
        self.fy = self.focal_length_mm / pixel_size_y

        # Principal point (image center)
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0

    def pixel_to_normalized(self, pixel_x: float, pixel_y: float) -> np.ndarray:
        """
        Convert pixel coordinates to normalized camera coordinates.
        
        Returns:
            ndarray: [x_norm, y_norm, 1.0] normalized coords (z=1 for rays)
        """
        x_norm = (pixel_x - self.cx) / self.fx
        y_norm = -(pixel_y - self.cy) / self.fy  # Negate: image y increases downward
        z_norm = 1.0

        return np.array([x_norm, y_norm, z_norm])

    def pixel_to_camera_ray(self, pixel_x: float, pixel_y: float) -> np.ndarray:
        """
        Convert pixel to normalized camera ray direction.
        
        Returns:
            ndarray: Normalized ray direction [dx, dy, dz]
        """
        ray = self.pixel_to_normalized(pixel_x, pixel_y)
        return ray / np.linalg.norm(ray)

    def get_sensor_info(self) -> dict:
        """Return camera specifications."""
        return {
            'image_width_px': self.image_width,
            'image_height_px': self.image_height,
            'focal_length_mm': self.focal_length_mm,
            'sensor_width_mm': self.sensor_width_mm,
            'sensor_height_mm': self.sensor_height_mm,
            'fx_pixels': f"{self.fx:.2f}",
            'fy_pixels': f"{self.fy:.2f}",
            'principal_point': f"({self.cx:.1f}, {self.cy:.1f})"
        }
