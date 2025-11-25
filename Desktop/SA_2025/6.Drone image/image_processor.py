"""
Image Processing Module
Handles drone image analysis using YOLOv8 for object detection.
Provides GSD (Ground Sample Distance) calculation for real-world measurements.
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict
from pathlib import Path


class ImageProcessor:
    """
    Processes drone images: loads, resizes, detects objects with YOLOv8.
    Calculates GSD for accurate real-world scale measurements.
    """

    def __init__(self, model_path: str = "yolov8n.pt"):
        """
        Initialize image processor with YOLOv8 model.
        
        Args:
            model_path: Path to YOLOv8 weights file
        """
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
        except ImportError:
            raise ImportError("YOLOv8 not installed. Install with: pip install ultralytics")
        
        self.last_image = None
        self.last_detections = None

    def load_image(self, image_path: str) -> np.ndarray:
        """
        Load image from file.
        
        Args:
            image_path: Path to image file
            
        Returns:
            ndarray: Image in BGR format (OpenCV standard)
        """
        image = cv2.imread(image_path)
        if image is None:
            raise FileNotFoundError(f"Image not found: {image_path}")
        
        self.last_image = image
        return image

    def calculate_gsd(self, altitude_m: float, focal_length_mm: float = 3.67, 
                      sensor_width_mm: float = 6.3, image_width_px: int = 4000) -> float:
        """
        Calculate Ground Sample Distance (GSD) - pixel size on ground.
        GSD = (altitude × sensor_width) / (focal_length × image_width)
        
        Args:
            altitude_m: Drone altitude above ground
            focal_length_mm: Camera focal length (default: 3.67 mm for DJI Mini 2)
            sensor_width_mm: Sensor width (default: 6.3 mm)
            image_width_px: Image width in pixels (default: 4000)
            
        Returns:
            float: GSD in meters/pixel
        """
        gsd = (altitude_m * sensor_width_mm) / (focal_length_mm * image_width_px)
        return gsd

    def detect_objects(self, image: np.ndarray, confidence: float = 0.5) -> Dict:
        """
        Detect objects using YOLOv8.
        
        Args:
            image: Input image (BGR format)
            confidence: Detection confidence threshold (0-1)
            
        Returns:
            dict: Detection results with bounding boxes, classes, confidence scores
        """
        results = self.model.predict(image, conf=confidence, verbose=False)
        
        detections = {
            'boxes': [],
            'classes': [],
            'confidences': [],
            'class_names': []
        }
        
        if results and len(results) > 0:
            result = results[0]
            if result.boxes is not None:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    detections['boxes'].append([int(x1), int(y1), int(x2), int(y2)])
                    detections['classes'].append(int(box.cls[0].cpu().numpy()))
                    detections['confidences'].append(float(box.conf[0].cpu().numpy()))
                    class_name = result.names[int(box.cls[0].cpu().numpy())]
                    detections['class_names'].append(class_name)
        
        self.last_detections = detections
        return detections

    def draw_detections(self, image: np.ndarray, detections: Dict, 
                       draw_labels: bool = True, thickness: int = 2) -> np.ndarray:
        """
        Draw detection boxes on image.
        
        Args:
            image: Input image
            detections: Dictionary from detect_objects()
            draw_labels: Whether to draw class names and confidence
            thickness: Box line thickness
            
        Returns:
            ndarray: Image with drawn detections
        """
        result_image = image.copy()
        
        for i, box in enumerate(detections['boxes']):
            x1, y1, x2, y2 = box
            cv2.rectangle(result_image, (x1, y1), (x2, y2), (0, 255, 0), thickness)
            
            if draw_labels:
                label = f"{detections['class_names'][i]} {detections['confidences'][i]:.2f}"
                cv2.putText(result_image, label, (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return result_image

    def get_object_center_pixel(self, box: List[int]) -> Tuple[float, float]:
        """
        Calculate center pixel coordinates of a detection box.
        
        Args:
            box: [x1, y1, x2, y2] bounding box
            
        Returns:
            tuple: (center_x, center_y) in pixels
        """
        x1, y1, x2, y2 = box
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        return center_x, center_y

    def pixel_to_enu_offset(self, center_x_px: float, center_y_px: float,
                            image_width_px: int, image_height_px: int,
                            gsd: float, altitude_m: float,
                            camera_pitch_deg: float = 90.0) -> Tuple[float, float]:
        """
        Convert pixel position to ENU offset assuming nadir view (straight down).
        Works for camera pitched 90° (perpendicular to ground).
        
        Args:
            center_x_px, center_y_px: Object center in pixels
            image_width_px, image_height_px: Image dimensions
            gsd: Ground Sample Distance (m/pixel)
            altitude_m: Drone altitude (unused here, built in GSD)
            camera_pitch_deg: Camera pitch angle (90° = nadir)
            
        Returns:
            tuple: (east_offset, north_offset) in meters from nadir point
        """
        # Normalize to image center (0,0) at center
        pixel_x_offset = center_x_px - image_width_px / 2.0
        pixel_y_offset = center_y_px - image_height_px / 2.0
        
        # Convert to meters (positive X = East, positive Y = North in image)
        east = pixel_x_offset * gsd
        north = pixel_y_offset * gsd
        
        return east, north

    def get_object_altitude(self, object_height_px: int, object_height_real_m: float,
                           gsd: float) -> float:
        """
        Estimate object altitude from pixel height (simplified).
        In reality, needs camera intrinsics and 3D reconstruction.
        
        Args:
            object_height_px: Object height in pixels
            object_height_real_m: Known real-world height
            gsd: Ground Sample Distance
            
        Returns:
            float: Altitude above ground (meters)
        """
        return object_height_px * gsd
