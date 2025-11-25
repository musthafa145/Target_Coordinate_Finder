#!/usr/bin/env python3
"""
Pixel-to-GPS Mapper for DJI Drone Images

This script maps pixel coordinates in a drone image to ground GPS coordinates
by extracting EXIF/XMP metadata and using camera geometry calculations.

Usage:
    python pixel_to_gps.py image.jpg [u] [v] [--sensor-width-mm MM]
    python pixel_to_gps.py image.jpg --interactive
    python pixel_to_gps.py --help
"""

import sys
import argparse
from pathlib import Path
from PIL import Image
from PIL.Image import Exif
import numpy as np
import xml.etree.ElementTree as ET


def get_metadata(img_path):
    """
    Extract EXIF and XMP metadata from drone image.
    
    Args:
        img_path (str): Path to image file
        
    Returns:
        dict: Metadata dictionary with fields like FocalLength, GPSLatitude, etc.
    """
    try:
        img = Image.open(img_path)
        exif = img.getexif()
    except FileNotFoundError:
        raise FileNotFoundError(f"❌ Image file not found: {img_path}")
    except Exception as e:
        raise RuntimeError(f"❌ Failed to open image: {e}")
    
    meta = {
        'img_width': img.width,
        'img_height': img.height,
        'FocalLength_mm': None,
        'GPSLatitude_deg': None,
        'GPSLongitude_deg': None,
        'GPSAltitude_m': None,
        'RelativeAltitude_m': None,
        'FlightRollDegree': None,
        'FlightPitchDegree': None,
        'FlightYawDegree': None,
        'GimbalRollDegree': None,
        'GimbalPitchDegree': None,
        'GimbalYawDegree': None,
        'DateTime': None,
    }
    
    # Extract EXIF focal length
    if exif:
        # Try main EXIF (tag 37386 is FocalLength)
        if 37386 in exif:
            focal_rational = exif[37386]
            meta['FocalLength_mm'] = float(focal_rational.numerator) / float(focal_rational.denominator)
        
        # Try Exif sub-IFD (tag 34665)
        if not meta['FocalLength_mm'] and 34665 in exif:
            try:
                exif_sub = exif.get_ifd(34665)
                if 37386 in exif_sub:
                    focal_rational = exif_sub[37386]
                    meta['FocalLength_mm'] = float(focal_rational.numerator) / float(focal_rational.denominator)
            except:
                pass
        
        # Extract GPS from IFD 34853 (GPS IFD)
        try:
            gps_ifd = exif.get_ifd(34853)
            if gps_ifd:
                # Tag 1: GPSLatitudeRef, Tag 2: GPSLatitude (tuple of 3 rationals)
                if 2 in gps_ifd:
                    lat_tuple = gps_ifd[2]
                    lat_deg = float(lat_tuple[0].numerator) / float(lat_tuple[0].denominator)
                    lat_min = float(lat_tuple[1].numerator) / float(lat_tuple[1].denominator)
                    lat_sec = float(lat_tuple[2].numerator) / float(lat_tuple[2].denominator)
                    lat = lat_deg + lat_min/60 + lat_sec/3600
                    
                    # Apply sign based on LatRef
                    if 1 in gps_ifd:
                        lat_ref = gps_ifd[1]
                        if lat_ref == 'S':
                            lat = -lat
                    meta['GPSLatitude_deg'] = lat
                
                # Tag 3: GPSLongitudeRef, Tag 4: GPSLongitude
                if 4 in gps_ifd:
                    lon_tuple = gps_ifd[4]
                    lon_deg = float(lon_tuple[0].numerator) / float(lon_tuple[0].denominator)
                    lon_min = float(lon_tuple[1].numerator) / float(lon_tuple[1].denominator)
                    lon_sec = float(lon_tuple[2].numerator) / float(lon_tuple[2].denominator)
                    lon = lon_deg + lon_min/60 + lon_sec/3600
                    
                    # Apply sign based on LonRef
                    if 3 in gps_ifd:
                        lon_ref = gps_ifd[3]
                        if lon_ref == 'W':
                            lon = -lon
                    meta['GPSLongitude_deg'] = lon
                
                # Tag 5: GPSAltitudeRef, Tag 6: GPSAltitude
                if 6 in gps_ifd:
                    alt_rational = gps_ifd[6]
                    meta['GPSAltitude_m'] = float(alt_rational.numerator) / float(alt_rational.denominator)
        except:
            pass
        
        # Extract DateTime
        if 306 in exif:
            meta['DateTime'] = exif[306]
    
    # Extract XMP drone-dji fields
    if 'exif' in img.info:
        xmp_raw = img.info.get('exif', b'')
        try:
            # Try to parse XMP from XMP segment (marker 0xFFE1)
            xmp_dict = img.getexif()
            # XMP is typically in a separate segment, try common locations
            if hasattr(img, 'info') and 'XMP' in img.info:
                xmp_str = img.info['XMP'].decode('utf-8') if isinstance(img.info['XMP'], bytes) else img.info['XMP']
                root = ET.fromstring(xmp_str)
                
                # Define XMP namespaces
                namespaces = {
                    'drone-dji': 'http://www.dji.com/drone-dji/1.0/',
                    'rdf': 'http://www.w3.org/1999/02/22-rdf-syntax-ns#',
                    'xmp': 'http://ns.adobe.com/xap/1.0/'
                }
                
                # Search for drone-dji elements
                for key in ['AbsoluteAltitude', 'RelativeAltitude', 'FlightRollDegree', 'FlightPitchDegree',
                           'FlightYawDegree', 'GimbalRollDegree', 'GimbalPitchDegree', 'GimbalYawDegree']:
                    for ns in ['drone-dji', '']:
                        xpath = f'.//{{{namespaces.get(ns, "")}}}' + key if ns else f'.//{key}'
                        elem = root.find(xpath)
                        if elem is not None and elem.text:
                            try:
                                meta[key] = float(elem.text)
                            except:
                                pass
        except:
            pass
    
    return meta


def compute_intrinsics(meta, sensor_width_mm):
    """
    Compute camera intrinsic matrix K from metadata and sensor specs.
    
    Args:
        meta (dict): Metadata with FocalLength_mm, img_width, img_height
        sensor_width_mm (float): Physical sensor width in mm
        
    Returns:
        tuple: (fx, fy, cx, cy) camera intrinsics in pixels
    """
    if not meta['FocalLength_mm']:
        raise ValueError("❌ Focal length not found in image metadata")
    
    f_mm = meta['FocalLength_mm']
    img_width = meta['img_width']
    img_height = meta['img_height']
    
    # Compute focal length in pixels: f_px = (f_mm / sensor_width_mm) * img_width
    fx = (f_mm / sensor_width_mm) * img_width
    fy = fx  # Assume square pixels
    
    # Principal point at image center
    cx = img_width / 2.0
    cy = img_height / 2.0
    
    return fx, fy, cx, cy


def pixel_to_normalized(u, v, fx, fy, cx, cy):
    """
    Convert pixel coordinates to normalized camera coordinates.
    
    Args:
        u, v (float): Pixel coordinates
        fx, fy, cx, cy (float): Camera intrinsics
        
    Returns:
        tuple: (x_norm, y_norm, z_norm=1) normalized ray direction
    """
    x_norm = (u - cx) / fx
    y_norm = -(v - cy) / fy  # Negate because image v increases downward
    z_norm = 1.0  # Optical axis
    
    return x_norm, y_norm, z_norm


def rotation_matrix_from_euler(yaw, pitch, roll):
    """
    Build rotation matrix from Euler angles (Z-Y-X convention).
    
    Args:
        yaw, pitch, roll (float): Angles in degrees
        
    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    yaw_rad = np.radians(yaw)
    pitch_rad = np.radians(pitch)
    roll_rad = np.radians(roll)
    
    # Rotation matrices for each axis
    Rz = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    
    Ry = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])
    
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])
    
    # Compose: R = Rz @ Ry @ Rx
    R = Rz @ Ry @ Rx
    return R


def build_rotation_matrix(meta):
    """
    Build combined rotation matrix (flight + gimbal).
    
    Args:
        meta (dict): Metadata with flight and gimbal angles
        
    Returns:
        np.ndarray: 3x3 rotation matrix (camera frame -> world frame)
    """
    # Flight attitude
    flight_yaw = meta['FlightYawDegree'] or 0
    flight_pitch = meta['FlightPitchDegree'] or 0
    flight_roll = meta['FlightRollDegree'] or 0
    
    # Gimbal attitude
    gimbal_yaw = meta['GimbalYawDegree'] or 0
    gimbal_pitch = meta['GimbalPitchDegree'] or 0
    gimbal_roll = meta['GimbalRollDegree'] or 0
    
    # Build rotation matrices
    R_flight = rotation_matrix_from_euler(flight_yaw, flight_pitch, flight_roll)
    R_gimbal = rotation_matrix_from_euler(gimbal_yaw, gimbal_pitch, gimbal_roll)
    
    # Combined: first apply gimbal rotation, then flight rotation
    R = R_flight @ R_gimbal
    return R


def intersect_ground_plane(origin, direction, ground_z=0):
    """
    Find intersection of ray with ground plane (z = ground_z).
    
    Args:
        origin (np.ndarray): Ray origin (3,)
        direction (np.ndarray): Ray direction (3,)
        ground_z (float): Ground plane z-coordinate
        
    Returns:
        tuple: (intersection_point, t_param) or (None, None) if no intersection
    """
    # Ray: P(t) = origin + t * direction
    # Plane: z = ground_z
    # Solve: origin[2] + t * direction[2] = ground_z
    
    if abs(direction[2]) < 1e-6:
        return None, None  # Ray parallel to ground
    
    t = (ground_z - origin[2]) / direction[2]
    
    if t < 0:
        return None, None  # Intersection behind ray origin
    
    intersection = origin + t * direction
    return intersection, t


def local_to_gps(lat0, lon0, east, north, R_earth=6371000):
    """
    Convert local ENU offset to GPS coordinates.
    
    Args:
        lat0, lon0 (float): Reference GPS in decimal degrees
        east, north (float): Local ENU offset in meters
        R_earth (float): Earth radius in meters
        
    Returns:
        tuple: (latitude, longitude) in decimal degrees
    """
    # Equirectangular approximation
    lat0_rad = np.radians(lat0)
    
    dlat = north / R_earth
    dlon = east / (R_earth * np.cos(lat0_rad))
    
    lat = lat0 + np.degrees(dlat)
    lon = lon0 + np.degrees(dlon)
    
    return lat, lon


def prompt_for_value(prompt_text, value_type=float, value_range=None, default=None):
    """
    Prompt user for input with validation.
    
    Args:
        prompt_text (str): Prompt message
        value_type (type): Expected type (int, float, str)
        value_range (tuple): Optional (min, max) range for validation
        default: Default value if user presses Enter
        
    Returns:
        Converted user input value
    """
    while True:
        try:
            user_input = input(prompt_text)
            
            if not user_input and default is not None:
                return default
            
            value = value_type(user_input)
            
            if value_range and (value < value_range[0] or value > value_range[1]):
                print(f"❌ Value out of range [{value_range[0]}, {value_range[1]}]. Try again.")
                continue
            
            return value
        except ValueError:
            print(f"❌ Invalid input. Expected {value_type.__name__}. Try again.")
        except KeyboardInterrupt:
            print("\n⏱️  Cancelled by user.")
            sys.exit(0)


def display_metadata_table(meta):
    """Display extracted metadata in a structured table."""
    print("\n" + "=" * 80)
    print("EXTRACTED DRONE METADATA")
    print("=" * 80 + "\n")
    
    print("Image:")
    print(f"  Width                                       {meta['img_width']}")
    print(f"  Height                                      {meta['img_height']}")
    if meta['FocalLength_mm']:
        print(f"  FocalLength_mm                            {meta['FocalLength_mm']:.4f}")
    else:
        print(f"  FocalLength_mm                            NOT FOUND")
    
    print("\nGPS Position:")
    if meta['GPSLatitude_deg'] and meta['GPSLongitude_deg']:
        print(f"  GPSLatitude_deg                            {meta['GPSLatitude_deg']:.6f}")
        print(f"  GPSLongitude_deg                           {meta['GPSLongitude_deg']:.6f}")
    else:
        print(f"  GPSLatitude_deg                            NOT FOUND")
        print(f"  GPSLongitude_deg                           NOT FOUND")
    
    print("\nAltitude:")
    if meta['RelativeAltitude_m']:
        print(f"  RelativeAltitude_m                         {meta['RelativeAltitude_m']:.4f}")
    elif meta['GPSAltitude_m']:
        print(f"  GPSAltitude_m                              {meta['GPSAltitude_m']:.4f}")
    else:
        print(f"  Altitude                                   NOT FOUND")
    
    print("\nFlight Attitude (degrees):")
    print(f"  FlightRollDegree                           {meta['FlightRollDegree'] or 0:.4f}")
    print(f"  FlightPitchDegree                          {meta['FlightPitchDegree'] or 0:.4f}")
    print(f"  FlightYawDegree                            {meta['FlightYawDegree'] or 0:.4f}")
    
    print("\nGimbal Attitude (degrees):")
    print(f"  GimbalRollDegree                           {meta['GimbalRollDegree'] or 0:.4f}")
    print(f"  GimbalPitchDegree                          {meta['GimbalPitchDegree'] or 0:.4f}")
    print(f"  GimbalYawDegree                            {meta['GimbalYawDegree'] or 0:.4f}")
    
    print("\n" + "=" * 80 + "\n")


def print_results(lat, lon, meta, pixel, local_offset):
    """Print final results in a formatted table."""
    print("\n" + "=" * 80)
    print("FINAL RESULTS")
    print("=" * 80 + "\n")
    
    print("🎯 TARGET GPS COORDINATES:")
    print(f"   Latitude:  {lat:.7f}°")
    print(f"   Longitude: {lon:.7f}°")
    
    print("\n📊 PROCESSING SUMMARY:")
    print(f"  Image: {meta['img_width']} x {meta['img_height']} pixels")
    print(f"  Focal length: {meta['FocalLength_mm']:.2f} mm")
    print(f"  Pixel target: ({pixel[0]:.1f}, {pixel[1]:.1f})")
    if local_offset:
        print(f"  Local offset from drone: E={local_offset[0]:+.2f}m, N={local_offset[1]:+.2f}m")
    if meta['GPSLatitude_deg'] and meta['GPSLongitude_deg']:
        print(f"  Drone GPS: ({meta['GPSLatitude_deg']:.7f}°, {meta['GPSLongitude_deg']:.7f}°)")
    alt = meta['RelativeAltitude_m'] or meta['GPSAltitude_m']
    if alt:
        print(f"  Drone altitude: {alt:.2f}m")
    
    print("\n" + "=" * 80)
    print("✅ Success!")
    print("=" * 80 + "\n")


def compute_ground_gps(img_path, u=None, v=None, sensor_width_mm=6.17, interactive=False):
    """
    Main computation: pixel -> ground GPS.
    
    Args:
        img_path (str): Image file path
        u, v (float): Pixel coordinates (optional if interactive)
        sensor_width_mm (float): Sensor width in mm
        interactive (bool): Use interactive prompts
        
    Returns:
        tuple: (latitude, longitude) of ground point
    """
    print("📷 Loading image metadata...")
    meta = get_metadata(img_path)
    
    # Display metadata table
    display_metadata_table(meta)
    
    # Validate required fields
    if not meta['GPSLatitude_deg'] or not meta['GPSLongitude_deg']:
        raise ValueError("❌ Drone GPS position not found in image metadata")
    
    if not meta['FocalLength_mm']:
        raise ValueError("❌ Focal length not found in image metadata. Try providing --sensor-width-mm.")
    
    alt = meta['RelativeAltitude_m'] or meta['GPSAltitude_m']
    if not alt:
        raise ValueError("❌ Drone altitude not found in image metadata")
    
    # Get pixel coordinates
    if interactive or u is None or v is None:
        print("📏 Sensor Width (mm):")
        print(f"  Default is 6.17 mm (DJI FC300S)")
        print(f"  Leave blank to use default, or enter custom value")
        sensor_width_mm = prompt_for_value("  > ", float, (1.0, 50.0), 6.17)
        
        print(f"\n📍 Target Pixel Coordinates:")
        print(f"  Image size: {meta['img_width']} x {meta['img_height']}")
        u = prompt_for_value(f"  Enter pixel u (x, 0 to {meta['img_width']}): ", float, (0, meta['img_width']))
        v = prompt_for_value(f"  Enter pixel v (y, 0 to {meta['img_height']}): ", float, (0, meta['img_height']))
    
    # Compute intrinsics
    print("\n⚙️  Computing camera intrinsics...")
    fx, fy, cx, cy = compute_intrinsics(meta, sensor_width_mm)
    print(f"   fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
    
    # Pixel to normalized
    print("🎯 Building camera ray...")
    x_norm, y_norm, z_norm = pixel_to_normalized(u, v, fx, fy, cx, cy)
    camera_ray = np.array([x_norm, y_norm, z_norm])
    camera_ray = camera_ray / np.linalg.norm(camera_ray)
    
    # Build rotation matrix
    print("🔄 Computing camera orientation...")
    R_cam_to_world = build_rotation_matrix(meta)
    
    # Transform ray to world frame
    print("🌍 Transforming ray to world coordinates...")
    world_ray = R_cam_to_world @ camera_ray
    
    # Drone position in world frame (ENU)
    drone_origin = np.array([0, 0, 0])  # At local origin initially
    
    # Ray origin at drone altitude above ground
    alt = meta['RelativeAltitude_m'] or meta['GPSAltitude_m']
    ray_origin = drone_origin + np.array([0, 0, alt])
    
    # Intersect with ground
    print("📍 Finding ground intersection...")
    intersection, t = intersect_ground_plane(ray_origin, world_ray, ground_z=0)
    
    if intersection is None:
        raise RuntimeError("❌ Ray does not intersect ground plane. Check pixel coordinates and gimbal angle.")
    
    east_offset = intersection[0]
    north_offset = intersection[1]
    
    print(f"   Local offset: E={east_offset:+.2f}m, N={north_offset:+.2f}m")
    
    # Convert to GPS
    print("📡 Converting to GPS coordinates...")
    lat, lon = local_to_gps(meta['GPSLatitude_deg'], meta['GPSLongitude_deg'], east_offset, north_offset)
    
    # Print results
    print_results(lat, lon, meta, (u, v), (east_offset, north_offset))
    
    return lat, lon


def main():
    """Parse arguments and run computation."""
    parser = argparse.ArgumentParser(
        description='Map pixel coordinates in drone images to ground GPS coordinates',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLES:
  # Interactive mode (prompts for all inputs)
  %(prog)s image.jpg --interactive

  # Command-line mode (all parameters provided)
  %(prog)s image.jpg 1920 1440 --sensor-width-mm 6.17

  # Custom sensor width
  %(prog)s image.jpg 2000 1500 --sensor-width-mm 6.0

NOTES:
  - GPS coordinates output to 7 decimal places (~0.11m accuracy)
  - Assumes flat ground at z=0 (local ENU frame)
  - Sensor width default: 6.17 mm (DJI FC300S); override for other cameras
  - Image must contain EXIF GPS data and focal length metadata
  - Flight attitude optional but improves accuracy
        """
    )
    
    parser.add_argument('image', nargs='?', help='Path to drone image file')
    parser.add_argument('u', nargs='?', type=float, help='Pixel u-coordinate (x)')
    parser.add_argument('v', nargs='?', type=float, help='Pixel v-coordinate (y)')
    parser.add_argument('--sensor-width-mm', type=float, default=6.17,
                       help='Sensor width in mm (default: 6.17 for DJI FC300S)')
    parser.add_argument('-i', '--interactive', action='store_true',
                       help='Use interactive mode with prompts')
    
    args = parser.parse_args()
    
    if not args.image:
        parser.print_help()
        sys.exit(1)
    
    try:
        lat, lon = compute_ground_gps(
            args.image,
            u=args.u,
            v=args.v,
            sensor_width_mm=args.sensor_width_mm,
            interactive=args.interactive
        )
        return 0
    except FileNotFoundError as e:
        print(f"\n❌ Error: {e}")
        return 1
    except ValueError as e:
        print(f"\n❌ Error: {e}")
        return 1
    except RuntimeError as e:
        print(f"\n❌ Error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n⏱️  Cancelled by user.")
        return 1
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
