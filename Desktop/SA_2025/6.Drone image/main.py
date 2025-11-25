"""
Main Application Module
Orchestrates the complete pixel-to-GPS target coordinate calculation pipeline.
Entry point for the Target Coordinate Calculator.
"""

import sys
import os
from pathlib import Path
import numpy as np
from metadata_parser import DJIMetadataParser
from camera_model import CameraModel
from geometry import GeometryEngine
from coordinate_transform import CoordinateTransform


class TargetCoordinateCalculator:
    """
    Main application class for converting image pixel coordinates to ground GPS coordinates.
    Integrates metadata parsing, camera modeling, geometry, and coordinate transformation.
    """

    def __init__(self, image_path: str):
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image file not found: {image_path}")

        self.image_path = image_path
        print(f"[INFO] Loading image: {image_path}")

        # Parse metadata
        self.parser = DJIMetadataParser(image_path)
        self.metadata = self.parser.get_all_metadata()

        # Validate metadata and print status
        self._validate_metadata()

        # Initialize camera with warnings if fields missing
        try:
            self.camera = CameraModel(self.metadata)
            sensor_info = self.camera.get_sensor_info()
            print("\n[CAMERA MODEL INFO]")
            for k, v in sensor_info.items():
                print(f"  {k}: {v}")
        except Exception as e:
            print(f"[WARNING] Camera model initialization failed: {e}")
            self.camera = None

        # Initialize other components safely
        try:
            altitude = self.metadata.get('altitude', 10.0)  # default 10 m
            gimbal_pitch = self.metadata.get('gimbal_pitch', 0.0)
            gimbal_roll = self.metadata.get('gimbal_roll', 0.0)
            gimbal_yaw = self.metadata.get('gimbal_yaw', 0.0)
            flight_yaw = self.metadata.get('flight_yaw', 0.0)

            self.geometry = GeometryEngine(
                drone_altitude_m=altitude,
                gimbal_pitch_deg=gimbal_pitch,
                gimbal_roll_deg=gimbal_roll,
                gimbal_yaw_deg=gimbal_yaw,
                flight_yaw_deg=flight_yaw
            )

            gps_lat = self.metadata.get('gps_latitude', 0.0)
            gps_lon = self.metadata.get('gps_longitude', 0.0)
            self.transformer = CoordinateTransform(
                ref_latitude_deg=gps_lat,
                ref_longitude_deg=gps_lon,
                ref_altitude_m=altitude
            )
        except Exception as e:
            print(f"[WARNING] Geometry/Transform initialization failed: {e}")
            self.geometry = None
            self.transformer = None

        print("[INFO] Calculator initialized (with warnings if any fields missing)")

    def _validate_metadata(self):
        required_fields = [
            'gps_latitude', 'gps_longitude', 'altitude',
            'gimbal_pitch', 'gimbal_roll', 'gimbal_yaw',
            'flight_yaw', 'focal_length_mm',
            'image_width', 'image_height'
        ]

        missing_fields = []
        available_fields = {}

        for field in required_fields:
            if field not in self.metadata or self.metadata[field] is None:
                missing_fields.append(field)
            else:
                available_fields[field] = self.metadata[field]

        print("\n[METADATA CHECK]")
        print("=" * 60)
        print("[AVAILABLE METADATA FIELDS]")
        for k, v in available_fields.items():
            print(f"  {k}: {v}")

        print("\n[MISSING METADATA FIELDS]")
        if missing_fields:
            for f in missing_fields:
                print(f"  {f}")
        else:
            print("  None (all required fields present)")
        print("=" * 60 + "\n")

        if missing_fields:
            print("[WARNING] Some metadata fields missing. Defaults will be used where applicable.\n")

    # -----------------------
    # Pixel-to-GPS conversion
    # -----------------------
    def pixel_to_gps(self, pixel_x: float, pixel_y: float) -> dict:
        if not self.camera or not self.geometry or not self.transformer:
            raise RuntimeError("Cannot compute GPS: missing critical components")

        img_w = self.metadata.get('image_width', 4000)
        img_h = self.metadata.get('image_height', 3000)

        if not (0 <= pixel_x < img_w and 0 <= pixel_y < img_h):
            raise ValueError(
                f"Pixel coordinates ({pixel_x}, {pixel_y}) out of bounds "
                f"(image: {img_w}x{img_h})"
            )

        print(f"\n[COMPUTE] Converting pixel ({pixel_x}, {pixel_y}) to GPS...")

        camera_ray = self.camera.pixel_to_camera_ray(pixel_x, pixel_y)
        print(f"  Camera ray: {camera_ray}")

        drone_position_enu = self.transformer.drone_to_enu(
            drone_lat_deg=self.metadata.get('gps_latitude', 0.0),
            drone_lon_deg=self.metadata.get('gps_longitude', 0.0),
            drone_alt_m=self.metadata.get('altitude', 10.0)
        )
        print(f"  Drone ENU position: {drone_position_enu}")

        # Transform camera ray into world frame (apply gimbal/flight rotations)
        world_ray = self.geometry.transform_camera_ray_to_world(camera_ray)
        print(f"  World ray: {world_ray}")

        intersection_enu = self.geometry.ray_ground_intersection(world_ray, drone_position_enu)
        if intersection_enu is None:
            print("[WARNING] Ray does not intersect ground. Returning None")
            return None

        print(f"  Ground intersection (ENU): {intersection_enu}")

        target_lat, target_lon = self.transformer.enu_to_gps(intersection_enu)
        print(f"  [SUCCESS] Target GPS: ({target_lat:.8f}, {target_lon:.8f})")

        return {
            'latitude': target_lat,
            'longitude': target_lon,
            'pixel_x': pixel_x,
            'pixel_y': pixel_y,
            'accuracy_note': 'Accuracy depends on metadata quality, flat-earth assumption (valid <1km)',
            'intermediate_data': {
                'camera_ray': camera_ray.tolist(),
                'drone_position_enu': drone_position_enu.tolist(),
                'intersection_enu': intersection_enu.tolist()
            }
        }

    def get_image_metadata(self) -> dict:
        return self.metadata.copy()

    def get_camera_info(self) -> dict:
        return self.camera.get_sensor_info() if self.camera else {}


# -----------------------
# Utility functions
# -----------------------
def print_banner():
    print("\n" + "="*70)
    print("  TARGET COORDINATE CALCULATOR (DJI Drone Image to GPS)")
    print("  Powered by: Pillow + NumPy (Flat-Earth Approximation)")
    print("="*70 + "\n")


def interactive_mode(calculator: TargetCoordinateCalculator):
    print("[MODE] Interactive pixel-to-GPS conversion")
    print("Enter pixel coordinates (x, y) to convert to GPS")
    print("Type 'exit' or 'quit' to exit\n")

    while True:
        try:
            user_input = input("Enter pixel coordinates (x y): ").strip().lower()
            if user_input in ['exit', 'quit']:
                print("[INFO] Exiting...")
                break
            coords = user_input.split()
            if len(coords) != 2:
                print("[ERROR] Enter two numbers separated by space")
                continue

            px, py = float(coords[0]), float(coords[1])
            result = calculator.pixel_to_gps(px, py)
            if result:
                print(f"\n  Result GPS Coordinates:")
                print(f"    Latitude:  {result['latitude']:.8f}°")
                print(f"    Longitude: {result['longitude']:.8f}°")
                print(f"    Note: {result['accuracy_note']}\n")
            else:
                print("[INFO] Could not compute GPS for this pixel.\n")

        except ValueError as e:
            print(f"[ERROR] {e}\n")
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted by user")
            break

    print("[INFO] Session ended")


def single_conversion_mode(calculator: TargetCoordinateCalculator, pixel_x: float, pixel_y: float):
    try:
        result = calculator.pixel_to_gps(pixel_x, pixel_y)
        if result:
            print("\n" + "="*70)
            print("RESULT")
            print("="*70)
            print(f"Pixel Input:     ({result['pixel_x']}, {result['pixel_y']})")
            print(f"GPS Output:      ({result['latitude']:.8f}°, {result['longitude']:.8f}°)")
            print(f"Accuracy Note:   {result['accuracy_note']}")
            print("="*70 + "\n")
        return result
    except Exception as e:
        print(f"[ERROR] {e}\n")
        return None


# -----------------------
# Entry point
# -----------------------
def main():
    print_banner()

    if len(sys.argv) < 2:
        print("[ERROR] Usage:")
        print("  Interactive mode:   python main.py <image_path>")
        print("  Single conversion:  python main.py <image_path> <pixel_x> <pixel_y>\n")
        sys.exit(1)

    image_path = sys.argv[1]

    try:
        calculator = TargetCoordinateCalculator(image_path)

        metadata = calculator.get_image_metadata()
        print("\n[METADATA] Drone Image Information:")
        print("="*70)
        print(f"  GPS Position:      ({metadata.get('gps_latitude', 0.0):.8f}°, {metadata.get('gps_longitude', 0.0):.8f}°)")
        print(f"  Altitude:          {metadata.get('altitude', 10.0):.2f} m")
        print(f"  Gimbal Angles:     Pitch={metadata.get('gimbal_pitch', 0.0):.1f}°, "
              f"Roll={metadata.get('gimbal_roll', 0.0):.1f}°, Yaw={metadata.get('gimbal_yaw', 0.0):.1f}°")
        print(f"  Flight Yaw:        {metadata.get('flight_yaw', 0.0):.1f}°")
        print(f"  Image Size:        {metadata.get('image_width', 4000)}x{metadata.get('image_height', 3000)} px")
        print(f"  Focal Length:      {metadata.get('focal_length_mm', 3.6):.1f} mm")
        print("="*70 + "\n")

        if len(sys.argv) == 4:
            pixel_x = float(sys.argv[2])
            pixel_y = float(sys.argv[3])
            single_conversion_mode(calculator, pixel_x, pixel_y)
        else:
            interactive_mode(calculator)

    except Exception as e:
        print(f"[ERROR] {e}\n")
        sys.exit(1)


if __name__ == '__main__':
    main()
