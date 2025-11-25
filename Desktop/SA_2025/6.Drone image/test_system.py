"""
Testing and Validation Script
Demonstrates usage of the drone image geo-referencing system.
Run examples, validate calculations, and test edge cases.
"""

import numpy as np
import sys
from pathlib import Path

# Import modules
from coordinate_transform import CoordinateTransform
from camera_model import CameraModel
from geometry import GeometryEngine


def test_coordinate_transform():
    """Test GPS ↔ ENU coordinate conversions."""
    print("\n" + "="*70)
    print("TEST 1: Coordinate Transformations (GPS ↔ ENU)")
    print("="*70)
    
    # Reference point (San Francisco Golden Gate Bridge)
    ref_lat, ref_lon, ref_alt = 37.8199, -122.4783, 0.0
    
    transformer = CoordinateTransform(ref_lat, ref_lon, ref_alt)
    
    # Test point: 100m east, 50m north
    test_enu = np.array([100.0, 50.0, 10.0])
    gps_lat, gps_lon = transformer.enu_to_gps(test_enu)
    
    print(f"Reference GPS: ({ref_lat:.6f}°, {ref_lon:.6f}°) @ {ref_alt}m")
    print(f"ENU offset: {test_enu}")
    print(f"Converted GPS: ({gps_lat:.6f}°, {gps_lon:.6f}°)")
    
    # Verify round-trip conversion
    enu_back = transformer.gps_to_enu(gps_lat, gps_lon, ref_alt + test_enu[2])
    error = np.linalg.norm(enu_back[:2] - test_enu[:2])
    print(f"Round-trip error: {error:.4f}m (should be <0.01m)")
    
    assert error < 0.01, "Coordinate transform round-trip failed!"
    print("✓ Coordinate transformation test PASSED")


def test_camera_model():
    """Test camera intrinsic parameters and pixel-to-ray conversion."""
    print("\n" + "="*70)
    print("TEST 2: Camera Model (Pixel → Ray Conversion)")
    print("="*70)
    
    # DJI Matrice 200 specifications
    metadata = {
        'image_width': 4000,
        'image_height': 3000,
        'focal_length_mm': 3.6
    }
    
    camera = CameraModel(metadata)
    print(f"Camera specs: {camera.get_sensor_info()}")
    
    # Test center pixel
    center_ray = camera.pixel_to_camera_ray(2000, 1500)
    print(f"\nCenter pixel (2000, 1500) ray: {center_ray}")
    assert np.isclose(center_ray[2], 1.0), "Center ray should have z≈1"
    assert np.abs(center_ray[0]) < 0.01, "Center ray X should be ~0"
    assert np.abs(center_ray[1]) < 0.01, "Center ray Y should be ~0"
    
    # Test corner pixel
    corner_ray = camera.pixel_to_camera_ray(0, 0)
    print(f"Corner pixel (0, 0) ray: {corner_ray}")
    magnitude = np.linalg.norm(corner_ray)
    assert np.isclose(magnitude, 1.0), f"Ray should be normalized, got {magnitude}"
    
    print("✓ Camera model test PASSED")


def test_geometry_engine():
    """Test 3D geometry operations and ray-ground intersections."""
    print("\n" + "="*70)
    print("TEST 3: Geometry Engine (Ray-Ground Intersections)")
    print("="*70)
    
    # Create geometry engine
    # Drone at 50m altitude, camera pointing straight down
    geometry = GeometryEngine(
        drone_altitude_m=50.0,
        gimbal_pitch_deg=-90.0,  # Straight down (nadir)
        gimbal_roll_deg=0.0,
        gimbal_yaw_deg=0.0,
        flight_yaw_deg=0.0
    )
    
    # Ray pointing straight down in camera frame
    camera_ray = np.array([0, 0, 1])
    camera_ray = camera_ray / np.linalg.norm(camera_ray)
    
    # Transform to world frame
    world_ray = geometry.transform_camera_ray_to_world(camera_ray)
    print(f"Camera ray: {camera_ray}")
    print(f"World ray: {world_ray}")
    
    # Drone position (at origin, 50m up)
    drone_enu = np.array([0, 0, 50])
    
    # Find ground intersection
    intersection = geometry.ray_ground_intersection(world_ray, drone_enu, ground_z=0)
    print(f"Drone position (ENU): {drone_enu}")
    print(f"Ground intersection: {intersection}")
    
    # Should intersect directly below drone
    assert intersection is not None
    assert np.isclose(intersection[0], 0, atol=0.1), f"X should be ~0, got {intersection[0]}"
    assert np.isclose(intersection[1], 0, atol=0.1), f"Y should be ~0, got {intersection[1]}"
    assert np.isclose(intersection[2], 0, atol=0.1), f"Z should be 0, got {intersection[2]}"
    
    print("✓ Geometry engine test PASSED")


def test_gsd_calculation():
    """Test Ground Sample Distance calculations."""
    print("\n" + "="*70)
    print("TEST 4: Ground Sample Distance (GSD) Calculation")
    print("="*70)
    
    from image_processor import ImageProcessor
    
    processor = ImageProcessor("yolov8n.pt")
    
    # Test scenarios
    test_cases = [
        (30.0, 3.67, 6.3, 4000, "30m altitude (low)"),
        (50.0, 3.67, 6.3, 4000, "50m altitude (medium)"),
        (100.0, 3.67, 6.3, 4000, "100m altitude (high)"),
    ]
    
    print(f"{'Altitude':<15} {'Focal':<12} {'GSD':<12} {'Description':<20}")
    print("-" * 60)
    
    for altitude, focal, sensor_w, img_w, desc in test_cases:
        gsd = processor.calculate_gsd(altitude, focal, sensor_w, img_w)
        print(f"{altitude:>6.1f}m{'':<8} {focal:>6.2f}mm{'':<5} {gsd:>10.4f} m/px {desc:<20}")
    
    # GSD should increase with altitude
    gsd_30 = processor.calculate_gsd(30.0, 3.67, 6.3, 4000)
    gsd_100 = processor.calculate_gsd(100.0, 3.67, 6.3, 4000)
    assert gsd_100 > gsd_30, "GSD should increase with altitude"
    
    print("✓ GSD calculation test PASSED")


def test_pixel_to_enu():
    """Test pixel coordinate to ENU conversion."""
    print("\n" + "="*70)
    print("TEST 5: Pixel → ENU Conversion (Nadir View)")
    print("="*70)
    
    from geo_referencer import GeoReferencer
    
    # Setup
    georeferencer = GeoReferencer(37.8199, -122.4783, 0.0)
    
    # Image properties
    image_width = 4000
    image_height = 3000
    gsd = 0.0127  # 50m altitude example
    
    # Test center pixel
    center_x, center_y = image_width / 2, image_height / 2
    east, north = georeferencer._pixel_to_enu_nadir(
        center_x, center_y, image_width, image_height, gsd
    )
    print(f"Center pixel ({center_x}, {center_y})")
    print(f"  ENU offset: ({east:.2f}m, {north:.2f}m)")
    assert np.isclose(east, 0, atol=0.01), "Center should map to (0,0)"
    assert np.isclose(north, 0, atol=0.01), "Center should map to (0,0)"
    
    # Test right edge
    right_x = image_width - 1
    east_r, north_r = georeferencer._pixel_to_enu_nadir(
        right_x, center_y, image_width, image_height, gsd
    )
    print(f"Right edge pixel ({right_x}, {center_y})")
    print(f"  ENU offset: ({east_r:.2f}m, {north_r:.2f}m)")
    assert east_r > 0, "Right edge should have positive east"
    
    # Test bottom edge
    bottom_y = image_height - 1
    east_b, north_b = georeferencer._pixel_to_enu_nadir(
        center_x, bottom_y, image_width, image_height, gsd
    )
    print(f"Bottom edge pixel ({center_x}, {bottom_y})")
    print(f"  ENU offset: ({east_b:.2f}m, {north_b:.2f}m)")
    assert north_b > 0, "Bottom edge should have positive north"
    
    print("✓ Pixel-to-ENU conversion test PASSED")


def test_yaw_rotation():
    """Test ENU rotation by drone yaw."""
    print("\n" + "="*70)
    print("TEST 6: ENU Rotation by Drone Yaw")
    print("="*70)
    
    from geo_referencer import GeoReferencer
    
    georeferencer = GeoReferencer(0.0, 0.0, 0.0)
    
    # Test cases
    test_cases = [
        (10, 0, 0, "Positive East (yaw=0°)"),
        (10, 0, 90, "Positive East rotated 90° → North"),
        (0, 10, 0, "Positive North (yaw=0°)"),
        (0, 10, 90, "Positive North rotated 90° → -East"),
    ]
    
    print(f"{'Input (E,N)':<15} {'Yaw':<8} {'Output (E,N)':<18} {'Description':<30}")
    print("-" * 70)
    
    for east, north, yaw, desc in test_cases:
        east_rot, north_rot = georeferencer._rotate_enu(east, north, yaw)
        print(f"({east:>3}, {north:>3}){'':<7} {yaw:>3}° {east_rot:>9.2f},{north_rot:>7.2f} {desc:<30}")
    
    # Verify 90° rotation
    east_r, north_r = georeferencer._rotate_enu(10, 0, 90)
    assert np.isclose(north_r, 10, atol=0.01), "90° rotation should swap E→N"
    assert np.isclose(east_r, 0, atol=0.01), "90° rotation should zero E"
    
    print("✓ Yaw rotation test PASSED")


def test_end_to_end():
    """End-to-end test: pixel → GPS."""
    print("\n" + "="*70)
    print("TEST 7: End-to-End Pixel → GPS Conversion")
    print("="*70)
    
    # Setup
    drone_lat, drone_lon, drone_alt = 37.8199, -122.4783, 50.0
    image_width, image_height = 4000, 3000
    focal_length = 3.6
    
    # Create components
    transformer = CoordinateTransform(drone_lat, drone_lon, 0.0)
    camera_meta = {
        'image_width': image_width,
        'image_height': image_height,
        'focal_length_mm': focal_length
    }
    camera = CameraModel(camera_meta)
    geometry = GeometryEngine(
        drone_altitude_m=drone_alt,
        gimbal_pitch_deg=-90.0,
        gimbal_yaw_deg=0.0
    )
    
    # Calculate GSD
    from image_processor import ImageProcessor
    processor = ImageProcessor("yolov8n.pt")
    gsd = processor.calculate_gsd(drone_alt, focal_length, 6.17, image_width)
    
    print(f"Drone position: ({drone_lat:.6f}°, {drone_lon:.6f}°) @ {drone_alt}m")
    print(f"GSD: {gsd:.6f} m/pixel")
    
    # Convert center pixel
    center_pixel_x, center_pixel_y = image_width / 2, image_height / 2
    
    # Step 1: Pixel → Camera ray
    camera_ray = camera.pixel_to_camera_ray(center_pixel_x, center_pixel_y)
    
    # Step 2: Camera ray → World ray
    world_ray = geometry.transform_camera_ray_to_world(camera_ray)
    
    # Step 3: Ray-ground intersection
    drone_enu = np.array([0, 0, drone_alt])
    intersection_enu = geometry.ray_ground_intersection(world_ray, drone_enu, ground_z=0)
    
    # Step 4: ENU → GPS
    target_lat, target_lon = transformer.enu_to_gps(intersection_enu)
    
    print(f"\nPixel ({center_pixel_x}, {center_pixel_y}):")
    print(f"  → Camera ray: {camera_ray}")
    print(f"  → World ray: {world_ray}")
    print(f"  → ENU intersection: {intersection_enu}")
    print(f"  → GPS: ({target_lat:.8f}°, {target_lon:.8f}°)")
    
    # For nadir view, center pixel should map to drone position
    dist = np.sqrt((target_lat - drone_lat)**2 + (target_lon - drone_lon)**2) * 111000  # approx meters
    print(f"\nDistance from drone: {dist:.2f}m (should be <1m for nadir)")
    assert dist < 1.0, f"Nadir view error too large: {dist:.2f}m"
    
    print("✓ End-to-end test PASSED")


def run_all_tests():
    """Run all tests."""
    print("\n" + "="*70)
    print("DRONE IMAGE GEO-REFERENCING SYSTEM - VALIDATION TEST SUITE")
    print("="*70)
    
    tests = [
        test_coordinate_transform,
        test_camera_model,
        test_geometry_engine,
        test_gsd_calculation,
        test_pixel_to_enu,
        test_yaw_rotation,
        test_end_to_end,
    ]
    
    passed = 0
    failed = 0
    
    for test_func in tests:
        try:
            test_func()
            passed += 1
        except Exception as e:
            print(f"✗ {test_func.__name__} FAILED: {e}")
            failed += 1
            import traceback
            traceback.print_exc()
    
    print("\n" + "="*70)
    print(f"TEST RESULTS: {passed} passed, {failed} failed out of {len(tests)} total")
    print("="*70 + "\n")
    
    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
