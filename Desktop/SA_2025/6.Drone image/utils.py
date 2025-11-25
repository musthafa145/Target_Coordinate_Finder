"""
Utility Functions and Helpers
Common operations for drone image geo-referencing.
"""

import os
import csv
import json
import numpy as np
from pathlib import Path
from typing import List, Dict, Tuple, Optional


class CoordinateUtils:
    """Utility functions for coordinate operations."""
    
    @staticmethod
    def decimal_to_dms(decimal_degrees: float) -> Tuple[int, int, float]:
        """
        Convert decimal degrees to degrees, minutes, seconds.
        
        Args:
            decimal_degrees: Decimal coordinate
            
        Returns:
            tuple: (degrees, minutes, seconds)
        """
        is_negative = decimal_degrees < 0
        decimal_degrees = abs(decimal_degrees)
        
        degrees = int(decimal_degrees)
        minutes_decimal = (decimal_degrees - degrees) * 60
        minutes = int(minutes_decimal)
        seconds = (minutes_decimal - minutes) * 60
        
        if is_negative:
            degrees = -degrees
        
        return degrees, minutes, seconds
    
    @staticmethod
    def dms_to_decimal(degrees: int, minutes: int, seconds: float) -> float:
        """
        Convert degrees, minutes, seconds to decimal.
        
        Args:
            degrees, minutes, seconds: DMS coordinates
            
        Returns:
            float: Decimal degrees
        """
        is_negative = degrees < 0
        degrees = abs(degrees)
        
        decimal = degrees + minutes / 60.0 + seconds / 3600.0
        
        if is_negative:
            decimal = -decimal
        
        return decimal
    
    @staticmethod
    def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate distance between two GPS points using Haversine formula.
        
        Args:
            lat1, lon1: First point (degrees)
            lat2, lon2: Second point (degrees)
            
        Returns:
            float: Distance in meters
        """
        R = 6371000  # Earth radius in meters
        
        lat1_rad = np.radians(lat1)
        lat2_rad = np.radians(lat2)
        dlat = np.radians(lat2 - lat1)
        dlon = np.radians(lon2 - lon1)
        
        a = np.sin(dlat / 2)**2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon / 2)**2
        c = 2 * np.arcsin(np.sqrt(a))
        
        return R * c
    
    @staticmethod
    def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate bearing (compass direction) from point 1 to point 2.
        
        Args:
            lat1, lon1: First point (degrees)
            lat2, lon2: Second point (degrees)
            
        Returns:
            float: Bearing in degrees (0° = North, 90° = East, etc.)
        """
        lat1_rad = np.radians(lat1)
        lat2_rad = np.radians(lat2)
        dlon = np.radians(lon2 - lon1)
        
        y = np.sin(dlon) * np.cos(lat2_rad)
        x = np.cos(lat1_rad) * np.sin(lat2_rad) - np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(dlon)
        
        bearing_rad = np.arctan2(y, x)
        bearing_deg = np.degrees(bearing_rad)
        
        # Normalize to 0-360
        bearing_deg = (bearing_deg + 360) % 360
        
        return bearing_deg


class ImageUtils:
    """Utility functions for image operations."""
    
    @staticmethod
    def get_image_dimensions(image_path: str) -> Tuple[int, int]:
        """
        Get image dimensions without loading entire image.
        
        Args:
            image_path: Path to image file
            
        Returns:
            tuple: (width, height) in pixels
        """
        import cv2
        image = cv2.imread(image_path)
        if image is None:
            raise FileNotFoundError(f"Image not found: {image_path}")
        height, width = image.shape[:2]
        return width, height
    
    @staticmethod
    def crop_image_to_roi(image_path: str, roi: Tuple[int, int, int, int], 
                         output_path: str) -> str:
        """
        Crop image to region of interest (ROI).
        
        Args:
            image_path: Input image path
            roi: (x1, y1, x2, y2) region
            output_path: Output image path
            
        Returns:
            str: Output path
        """
        import cv2
        image = cv2.imread(image_path)
        x1, y1, x2, y2 = roi
        cropped = image[y1:y2, x1:x2]
        cv2.imwrite(output_path, cropped)
        return output_path
    
    @staticmethod
    def create_gps_overlay(detections: List[Dict], image_path: str, 
                          output_path: str) -> str:
        """
        Create visualization with GPS coordinates overlaid on image.
        
        Args:
            detections: List of detection dicts with GPS data
            image_path: Input image path
            output_path: Output visualization path
            
        Returns:
            str: Output path
        """
        import cv2
        
        image = cv2.imread(image_path)
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        for det in detections:
            # Draw box
            box = det.get('pixel_box', [])
            if box and len(box) == 4:
                x1, y1, x2, y2 = box
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw GPS text
            gps_lat = det.get('gps_latitude', 0)
            gps_lon = det.get('gps_longitude', 0)
            text = f"{gps_lat:.6f}, {gps_lon:.6f}"
            
            if box:
                cv2.putText(image, text, (x1, y1 - 10),
                           font, 0.4, (0, 255, 0), 1)
        
        cv2.imwrite(output_path, image)
        return output_path


class DataUtils:
    """Utility functions for data processing and export."""
    
    @staticmethod
    def load_csv(file_path: str) -> List[Dict]:
        """Load CSV file to list of dictionaries."""
        results = []
        with open(file_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                results.append(row)
        return results
    
    @staticmethod
    def save_csv(data: List[Dict], file_path: str, fieldnames: Optional[List[str]] = None):
        """Save list of dictionaries to CSV."""
        if not data:
            print(f"No data to save to {file_path}")
            return
        
        if fieldnames is None:
            fieldnames = list(data[0].keys())
        
        with open(file_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data)
    
    @staticmethod
    def load_json(file_path: str) -> Dict:
        """Load JSON file."""
        with open(file_path, 'r') as f:
            return json.load(f)
    
    @staticmethod
    def save_json(data: Dict, file_path: str):
        """Save dictionary to JSON."""
        with open(file_path, 'w') as f:
            json.dump(data, f, indent=2)
    
    @staticmethod
    def merge_detections(detection_files: List[str], output_file: str) -> int:
        """
        Merge multiple detection CSV files into one.
        
        Args:
            detection_files: List of CSV file paths
            output_file: Output merged CSV path
            
        Returns:
            int: Total number of detections merged
        """
        all_detections = []
        
        for file_path in detection_files:
            detections = DataUtils.load_csv(file_path)
            all_detections.extend(detections)
        
        if all_detections:
            DataUtils.save_csv(all_detections, output_file)
        
        return len(all_detections)
    
    @staticmethod
    def filter_detections(detections: List[Dict], 
                         min_confidence: float = 0.0,
                         object_classes: Optional[List[str]] = None) -> List[Dict]:
        """
        Filter detections by confidence and/or class.
        
        Args:
            detections: List of detection dicts
            min_confidence: Minimum confidence threshold
            object_classes: List of allowed classes (None = all)
            
        Returns:
            list: Filtered detections
        """
        filtered = []
        
        for det in detections:
            # Check confidence
            conf = float(det.get('Confidence', 0))
            if conf < min_confidence:
                continue
            
            # Check class
            if object_classes:
                obj_class = det.get('Object', '')
                if obj_class not in object_classes:
                    continue
            
            filtered.append(det)
        
        return filtered
    
    @staticmethod
    def compute_statistics(detections: List[Dict]) -> Dict:
        """
        Compute statistics from detections.
        
        Args:
            detections: List of detection dicts
            
        Returns:
            dict: Statistics (count, confidence stats, location bounds)
        """
        if not detections:
            return {}
        
        confidences = [float(d.get('Confidence', 0)) for d in detections]
        latitudes = [float(d.get('Latitude', 0)) for d in detections]
        longitudes = [float(d.get('Longitude', 0)) for d in detections]
        
        stats = {
            'total_detections': len(detections),
            'confidence': {
                'mean': np.mean(confidences),
                'min': np.min(confidences),
                'max': np.max(confidences),
                'std': np.std(confidences)
            },
            'location_bounds': {
                'lat_min': np.min(latitudes),
                'lat_max': np.max(latitudes),
                'lon_min': np.min(longitudes),
                'lon_max': np.max(longitudes)
            },
            'location_center': {
                'latitude': np.mean(latitudes),
                'longitude': np.mean(longitudes)
            }
        }
        
        # Count by class
        classes = {}
        for det in detections:
            obj_class = det.get('Object', 'Unknown')
            classes[obj_class] = classes.get(obj_class, 0) + 1
        stats['classes'] = classes
        
        return stats


class ValidationUtils:
    """Utility functions for validating data and parameters."""
    
    @staticmethod
    def validate_gps(latitude: float, longitude: float) -> bool:
        """
        Validate GPS coordinates are within valid ranges.
        
        Args:
            latitude, longitude: GPS coordinates
            
        Returns:
            bool: True if valid
        """
        return -90 <= latitude <= 90 and -180 <= longitude <= 180
    
    @staticmethod
    def validate_pixel(pixel_x: float, pixel_y: float, 
                      image_width: int, image_height: int) -> bool:
        """
        Validate pixel coordinates are within image bounds.
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            image_width, image_height: Image dimensions
            
        Returns:
            bool: True if valid
        """
        return 0 <= pixel_x < image_width and 0 <= pixel_y < image_height
    
    @staticmethod
    def validate_altitude(altitude: float) -> bool:
        """Validate altitude is positive and reasonable."""
        return 0 < altitude < 10000  # 0-10km range


class ReportGenerator:
    """Generate reports from detection results."""
    
    @staticmethod
    def generate_html_report(detections: List[Dict], 
                            image_path: str,
                            output_path: str) -> str:
        """
        Generate HTML report with detections and map.
        
        Args:
            detections: List of detection dicts with GPS
            image_path: Path to drone image
            output_path: Output HTML path
            
        Returns:
            str: Output path
        """
        stats = DataUtils.compute_statistics(detections)
        
        html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Drone Detection Report</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .stat {{ margin: 10px 0; }}
        table {{ border-collapse: collapse; width: 100%; margin-top: 20px; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background-color: #f2f2f2; }}
    </style>
</head>
<body>
    <h1>Drone Image Detection Report</h1>
    
    <h2>Summary Statistics</h2>
    <div class="stat"><strong>Total Detections:</strong> {stats.get('total_detections', 0)}</div>
    <div class="stat"><strong>Mean Confidence:</strong> {stats.get('confidence', {}).get('mean', 0):.2%}</div>
    <div class="stat"><strong>Location Center:</strong> {stats.get('location_center', {}).get('latitude', 0):.6f}°, {stats.get('location_center', {}).get('longitude', 0):.6f}°</div>
    
    <h2>Detections by Class</h2>
    <table>
        <tr>
            <th>Class</th>
            <th>Count</th>
        </tr>
"""
        for obj_class, count in stats.get('classes', {}).items():
            html += f"        <tr><td>{obj_class}</td><td>{count}</td></tr>\n"
        
        html += """
    </table>
    
    <h2>All Detections</h2>
    <table>
        <tr>
            <th>Object</th>
            <th>Confidence</th>
            <th>Latitude</th>
            <th>Longitude</th>
        </tr>
"""
        for det in detections:
            html += f"""        <tr>
            <td>{det.get('Object', '')}</td>
            <td>{det.get('Confidence', '')}</td>
            <td>{det.get('Latitude', '')}</td>
            <td>{det.get('Longitude', '')}</td>
        </tr>
"""
        
        html += """
    </table>
</body>
</html>
"""
        
        with open(output_path, 'w') as f:
            f.write(html)
        
        return output_path


# Main execution for testing
if __name__ == "__main__":
    print("Coordinate Utilities Test")
    print("-" * 40)
    
    # Test DMS conversion
    decimal = 40.7128
    d, m, s = CoordinateUtils.decimal_to_dms(decimal)
    print(f"Decimal {decimal}° = {d}° {m}' {s:.1f}\"")
    
    decimal_back = CoordinateUtils.dms_to_decimal(d, m, s)
    print(f"Back to decimal: {decimal_back}°")
    
    # Test distance calculation
    lat1, lon1 = 40.7128, -74.0060  # NYC
    lat2, lon2 = 34.0522, -118.2437  # LA
    dist = CoordinateUtils.haversine_distance(lat1, lon1, lat2, lon2)
    print(f"\nDistance NYC-LA: {dist:,.0f} m ({dist/1000:.1f} km)")
    
    # Test bearing
    bearing = CoordinateUtils.bearing(lat1, lon1, lat2, lon2)
    print(f"Bearing NYC→LA: {bearing:.1f}°")
    
    # Test validation
    print(f"\nValidate GPS (40.7128, -74.0060): {ValidationUtils.validate_gps(40.7128, -74.0060)}")
    print(f"Validate GPS (100, 200): {ValidationUtils.validate_gps(100, 200)}")
    print(f"Validate pixel (1920, 1080, 4000, 3000): {ValidationUtils.validate_pixel(1920, 1080, 4000, 3000)}")
    print(f"Validate altitude (50.0m): {ValidationUtils.validate_altitude(50.0)}")
