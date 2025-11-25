"""
Practical Examples
Demonstrates real-world usage scenarios for the drone image geo-referencing system.
"""

import numpy as np
from coordinate_transform import CoordinateTransform
from camera_model import CameraModel
from geometry import GeometryEngine
from image_processor import ImageProcessor
from geo_referencer import GeoReferencer, DroneState


def example_1_simple_pixel_to_gps():
    """
    Example 1: Simple pixel-to-GPS conversion
    Scenario: You see an object on your drone display at pixel (1920, 1080)
    and want to find its GPS coordinates.
    """
    print("\n" + "="*70)
    print("EXAMPLE 1: Convert Pixel to GPS Coordinates")
    print("="*70)
    
    # Drone telemetry
    drone_lat = 37.8199
    drone_lon = -122.4783
    drone_alt = 50.0  # meters
    
    # Image properties
    image_width = 4000
    image_height = 3000
    focal_length = 3.6  # mm
    
    # Setup components
    transformer = CoordinateTransform(drone_lat, drone_lon, 0.0)
    camera_meta = {
        'image_width': image_width,
        'image_height': image_height,
        'focal_length_mm': focal_length
    }
    camera = CameraModel(camera_meta)
    geometry = GeometryEngine(
        drone_altitude_m=drone_alt,
        gimbal_pitch_deg=-90.0,  # Camera straight down
        gimbal_yaw_deg=0.0
    )
    
    # Target pixel
    target_pixel_x = 1920
    target_pixel_y = 1080
    
    print(f"Drone Position: ({drone_lat:.6f}°, {drone_lon:.6f}°) @ {drone_alt}m altitude")
    print(f"Target Pixel: ({target_pixel_x}, {target_pixel_y})")
    
    # Conversion pipeline
    camera_ray = camera.pixel_to_camera_ray(target_pixel_x, target_pixel_y)
    world_ray = geometry.transform_camera_ray_to_world(camera_ray)
    drone_enu = np.array([0, 0, drone_alt])
    intersection_enu = geometry.ray_ground_intersection(world_ray, drone_enu, ground_z=0)
    
    if intersection_enu is not None:
        target_lat, target_lon = transformer.enu_to_gps(intersection_enu)
        print(f"\nResult:")
        print(f"  Target GPS: ({target_lat:.8f}°, {target_lon:.8f}°)")
        print(f"  ENU Offset: ({intersection_enu[0]:.1f}m E, {intersection_enu[1]:.1f}m N)")
    else:
        print("ERROR: Ray does not intersect ground!")


def example_2_multiple_pixel_conversion():
    """
    Example 2: Convert multiple pixels efficiently
    Scenario: Identify several points of interest in the image.
    """
    print("\n" + "="*70)
    print("EXAMPLE 2: Convert Multiple Pixels")
    print("="*70)
    
    # Setup
    drone_lat, drone_lon, drone_alt = 40.7128, -74.0060, 75.0
    image_width, image_height = 4000, 3000
    focal_length = 3.6
    
    transformer = CoordinateTransform(drone_lat, drone_lon, 0.0)
    camera = CameraModel({
        'image_width': image_width,
        'image_height': image_height,
        'focal_length_mm': focal_length
    })
    geometry = GeometryEngine(drone_altitude_m=drone_alt, gimbal_pitch_deg=-90.0)
    
    # Points of interest
    poi_list = [
        (1000, 1000, "Building 1"),
        (2000, 1500, "Building 2"),
        (3000, 2000, "Park"),
    ]
    
    print(f"Drone: ({drone_lat:.4f}°, {drone_lon:.4f}°) @ {drone_alt}m")
    print(f"\n{'#':<3} {'Pixel Location':<20} {'GPS Coordinates':<30} {'ENU':<30}")
    print("-" * 85)
    
    for i, (px, py, label) in enumerate(poi_list, 1):
        camera_ray = camera.pixel_to_camera_ray(px, py)
        world_ray = geometry.transform_camera_ray_to_world(camera_ray)
        drone_enu = np.array([0, 0, drone_alt])
        intersection_enu = geometry.ray_ground_intersection(world_ray, drone_enu)
        
        if intersection_enu is not None:
            target_lat, target_lon = transformer.enu_to_gps(intersection_enu)
            enu_str = f"({intersection_enu[0]:.1f}, {intersection_enu[1]:.1f})"
            print(f"{i:<3} ({px}, {py})           "
                  f"({target_lat:.6f}, {target_lon:.6f})  {enu_str:<30} {label}")


def example_3_gsd_and_scale():
    """
    Example 3: Calculate actual ground distances from pixel measurements
    Scenario: Measure the width of a road or building length in pixels
    and convert to real meters.
    """
    print("\n" + "="*70)
    print("EXAMPLE 3: Calculate Real-World Distances from Pixels")
    print("="*70)
    
    processor = ImageProcessor("yolov8n.pt")
    
    # Measurement in pixels
    pixel_width = 150  # Building width in pixels
    pixel_length = 250  # Building length in pixels
    
    # Drone altitude affects scale
    altitudes = [30, 50, 75, 100]
    focal_length = 3.6
    sensor_width = 6.17
    image_width = 4000
    
    print(f"Object measured: {pixel_width}px width × {pixel_length}px length")
    print(f"\n{'Altitude':<12} {'GSD':<12} {'Real Width':<15} {'Real Length':<15}")
    print("-" * 55)
    
    for alt in altitudes:
        gsd = processor.calculate_gsd(alt, focal_length, sensor_width, image_width)
        real_width = pixel_width * gsd
        real_length = pixel_length * gsd
        print(f"{alt:>6.0f}m{'':<5} {gsd:>8.5f}m/px {real_width:>11.2f}m {real_length:>15.2f}m")


def example_4_drone_yaw_rotation():
    """
    Example 4: Account for drone yaw when converting pixels
    Scenario: Drone is rotated (not aligned with cardinal directions).
    """
    print("\n" + "="*70)
    print("EXAMPLE 4: Handle Drone Yaw Rotation")
    print("="*70)
    
    drone_lat, drone_lon, drone_alt = 39.7392, -104.9903, 50.0
    
    # Setup
    transformer = CoordinateTransform(drone_lat, drone_lon, 0.0)
    camera = CameraModel({
        'image_width': 4000,
        'image_height': 3000,
        'focal_length_mm': 3.6
    })
    
    # Pixel location
    px, py = 2000, 1500  # Center of image (nadir point)
    
    print(f"Drone at: ({drone_lat:.6f}°, {drone_lon:.6f}°) @ {drone_alt}m")
    print(f"Pixel: ({px}, {py})")
    print(f"\n{'Yaw Angle':<12} {'Latitude':<18} {'Longitude':<18} {'Note':<25}")
    print("-" * 75)
    
    yaw_angles = [0, 45, 90, 180, 270]
    
    for yaw in yaw_angles:
        # Create geometry with yaw
        geometry = GeometryEngine(
            drone_altitude_m=drone_alt,
            gimbal_pitch_deg=-90.0,
            gimbal_yaw_deg=yaw
        )
        
        camera_ray = camera.pixel_to_camera_ray(px, py)
        world_ray = geometry.transform_camera_ray_to_world(camera_ray)
        drone_enu = np.array([0, 0, drone_alt])
        intersection_enu = geometry.ray_ground_intersection(world_ray, drone_enu)
        
        if intersection_enu is not None:
            target_lat, target_lon = transformer.enu_to_gps(intersection_enu)
            
            # For nadir (center pixel), rotation shouldn't matter much
            # But gimbal yaw does affect the frame
            dist_from_drone = np.sqrt(intersection_enu[0]**2 + intersection_enu[1]**2)
            direction = np.degrees(np.arctan2(intersection_enu[1], intersection_enu[0]))
            
            note = f"Direction: {direction:.1f}°"
            print(f"{yaw:>5.0f}°{'':<6} {target_lat:>14.8f}° {target_lon:>16.8f}° {note:<25}")


def example_5_batch_object_detection():
    """
    Example 5: Detect objects and get their GPS coordinates
    Scenario: Process an image with YOLOv8 and get GPS of detected objects.
    (Requires actual image file)
    """
    print("\n" + "="*70)
    print("EXAMPLE 5: Batch Object Detection and Geo-Referencing")
    print("="*70)
    print("(This example requires an actual image file)")
    
    # Simulated detections (in real code, these come from YOLOv8)
    simulated_detections = {
        'boxes': [
            [1000, 800, 1200, 1100],   # Person
            [2500, 1200, 2800, 2000],  # Car
            [3000, 500, 3300, 900],    # Dog
        ],
        'classes': [0, 2, 16],
        'confidences': [0.95, 0.88, 0.72],
        'class_names': ['person', 'car', 'dog']
    }
    
    # Drone state
    drone_state = DroneState(
        latitude_deg=48.8566,
        longitude_deg=2.3522,
        altitude_m=60.0,
        yaw_deg=0.0
    )
    
    # Create geo-referencer
    georeferencer = GeoReferencer(
        drone_state.latitude_deg,
        drone_state.longitude_deg,
        0.0
    )
    
    # Calculate GSD
    processor = ImageProcessor("yolov8n.pt")
    gsd = processor.calculate_gsd(drone_state.altitude_m)
    
    print(f"Drone: ({drone_state.latitude_deg:.6f}°, {drone_state.longitude_deg:.6f}°) @ {drone_state.altitude_m}m")
    print(f"GSD: {gsd:.5f} m/pixel")
    print(f"\n{'#':<3} {'Object':<10} {'Confidence':<12} {'Latitude':<15} {'Longitude':<15}")
    print("-" * 60)
    
    # Convert detections to GPS
    geo_objects = georeferencer.batch_detect_to_gps(
        simulated_detections, drone_state,
        4000, 3000,  # Image dimensions
        gsd
    )
    
    for i, obj in enumerate(geo_objects, 1):
        conf_str = f"{obj.confidence:.1%}"
        print(f"{i:<3} {obj.class_name:<10} {conf_str:<12} "
              f"{obj.gps_latitude:>14.8f}° {obj.gps_longitude:>15.8f}°")


def example_6_gps_to_pixel():
    """
    Example 6: Reverse operation - convert GPS to pixel coordinates
    Scenario: You know a target's GPS coordinates and want to find
    it in the drone image.
    """
    print("\n" + "="*70)
    print("EXAMPLE 6: Convert GPS Coordinates to Pixel Location")
    print("="*70)
    
    drone_lat, drone_lon, drone_alt = 35.6762, 139.6503, 80.0  # Tokyo, 80m alt
    image_width, image_height = 4000, 3000
    
    # Target GPS location (near drone)
    target_lat = drone_lat + 0.0001  # ~11m north
    target_lon = drone_lon + 0.0002  # ~15m east
    
    # Setup
    transformer = CoordinateTransform(drone_lat, drone_lon, 0.0)
    
    # Convert target GPS to ENU
    target_enu = transformer.gps_to_enu(target_lat, target_lon, drone_alt)
    
    print(f"Drone: ({drone_lat:.6f}°, {drone_lon:.6f}°) @ {drone_alt}m")
    print(f"Target: ({target_lat:.6f}°, {target_lon:.6f}°)")
    print(f"Target ENU offset: E={target_enu[0]:.1f}m, N={target_enu[1]:.1f}m")
    
    # Calculate GSD
    processor = ImageProcessor("yolov8n.pt")
    focal_length = 3.6
    sensor_width = 6.17
    gsd = processor.calculate_gsd(drone_alt, focal_length, sensor_width, image_width)
    
    # Convert ENU to pixel
    # Note: This is a simplified conversion for nadir view
    pixel_x_offset = target_enu[0] / gsd
    pixel_y_offset = -target_enu[1] / gsd  # Note: negative because image Y is inverted
    
    pixel_x = image_width / 2 + pixel_x_offset
    pixel_y = image_height / 2 + pixel_y_offset
    
    print(f"\nGSD: {gsd:.5f} m/pixel")
    print(f"Estimated pixel location: ({pixel_x:.0f}, {pixel_y:.0f})")
    
    # Check if within bounds
    if 0 <= pixel_x < image_width and 0 <= pixel_y < image_height:
        print("✓ Target is visible in the image")
    else:
        print("✗ Target is outside the image bounds")


def run_all_examples():
    """Run all examples."""
    print("\n" + "="*70)
    print("DRONE IMAGE GEO-REFERENCING - PRACTICAL EXAMPLES")
    print("="*70)
    
    examples = [
        example_1_simple_pixel_to_gps,
        example_2_multiple_pixel_conversion,
        example_3_gsd_and_scale,
        example_4_drone_yaw_rotation,
        example_5_batch_object_detection,
        example_6_gps_to_pixel,
    ]
    
    for example_func in examples:
        try:
            example_func()
        except Exception as e:
            print(f"\nERROR in {example_func.__name__}: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "="*70)
    print("All examples completed!")
    print("="*70 + "\n")


if __name__ == "__main__":
    run_all_examples()
