# Quick Start Guide - Drone Image Geo-Referencing System

## 5-Minute Setup

### 1. Install Dependencies
```bash
pip install opencv-python numpy pillow ultralytics exifread
```

### 2. Verify Installation
```bash
python -c "import cv2, numpy, PIL, ultralytics; print('All dependencies installed!')"
```

### 3. Run Tests
```bash
python test_system.py
```

## Usage Scenarios

### Scenario A: I have a drone image and want GPS for a specific pixel

**Option 1: Interactive mode (no parameters needed)**
```bash
python main.py drone_photo.jpg
```
Then type pixel coordinates when prompted:
```
Enter pixel coordinates (x y): 1920 1080
Result GPS Coordinates:
  Latitude:  37.123456°
  Longitude: -122.456789°
```

**Option 2: Command-line single conversion**
```bash
python main.py drone_photo.jpg 1920 1080
```

### Scenario B: I want to detect objects and get their GPS locations

Assuming you have an image and know the drone's state:
```bash
python main.py photo.jpg --detect \
    --drone-lat 37.123456 \
    --drone-lon -122.456789 \
    --altitude 50.0 \
    --output ./results
```

### Scenario C: I want to process multiple images in a batch

Create a Python script:
```python
from main import TargetCoordinateCalculator
import os

image_folder = "./drone_images/"
results = {}

for image_file in os.listdir(image_folder):
    if image_file.endswith('.jpg'):
        calc = TargetCoordinateCalculator(os.path.join(image_folder, image_file))
        
        # Convert several pixels of interest
        result = calc.pixel_to_gps(1920, 1080)  # center
        results[image_file] = result
        
        print(f"{image_file}: {result['latitude']}, {result['longitude']}")
```

### Scenario D: I'm using YOLOv8 for detection but need GPS coordinates

```python
from image_processor import ImageProcessor
from geo_referencer import GeoReferencer, DroneState
import cv2

# Load image and detect
processor = ImageProcessor("yolov8n.pt")
image = processor.load_image("drone_photo.jpg")
detections = processor.detect_objects(image, confidence=0.6)

# Create geo-referencer
georeferencer = GeoReferencer(
    ref_latitude_deg=37.123456,
    ref_longitude_deg=-122.456789,
    ref_altitude_m=0.0
)

# Create drone state
drone_state = DroneState(
    latitude_deg=37.123456,
    longitude_deg=-122.456789,
    altitude_m=50.0,
    yaw_deg=0.0  # Camera aligned north
)

# Calculate GSD and convert
gsd = processor.calculate_gsd(drone_state.altitude_m)
geo_objects = georeferencer.batch_detect_to_gps(
    detections, drone_state,
    image.shape[1], image.shape[0],  # width, height
    gsd
)

# Print results
for obj in geo_objects:
    print(f"{obj.class_name}: {obj.gps_latitude:.8f}, {obj.gps_longitude:.8f}")

# Export
georeferencer.export_detections_csv(geo_objects, "objects.csv")
georeferencer.export_detections_json(geo_objects, "objects.json")
```

## Common Issues & Solutions

### "Image file not found"
- **Cause**: Incorrect file path
- **Solution**: Use absolute path or check file exists:
  ```bash
  ls -la drone_photo.jpg
  ```

### "EXIF metadata missing"
- **Cause**: Image doesn't have GPS/altitude embedded
- **Solution**: Provide parameters manually:
  ```bash
  python main.py photo.jpg --drone-lat 37.123 --drone-lon -122.456 --altitude 50
  ```

### "YOLOv8 not installed"
- **Cause**: Missing ultralytics package
- **Solution**: Install it:
  ```bash
  pip install ultralytics
  ```
  First run will download the model (~500MB)

### "Coordinate seems wrong"
- **Cause**: Gimbal calibration issue or flat-earth model breaking down
- **Solution**:
  1. Verify drone altitude is correct
  2. For distances > 1km, use WGS84 model instead
  3. Check gimbal pitch/roll angles in metadata

### "Python module not found"
- **Cause**: Missing import or circular dependency
- **Solution**: Ensure all .py files are in the same directory:
  ```bash
  ls *.py  # Should show all modules
  ```

## Understanding Key Concepts

### GSD (Ground Sample Distance)
- **Definition**: How many meters per pixel
- **Formula**: GSD = (Altitude × Sensor_Width) / (Focal_Length × Image_Width)
- **Impact**: Higher altitude = larger GSD = less detail
- **Example**: At 50m altitude with standard DJI camera: GSD ≈ 0.0127 m/pixel (1.27 cm/pixel)

### Coordinate Systems
- **GPS**: Latitude/Longitude (WGS84)
- **ENU**: East-North-Up local Cartesian frame
  - East: positive X (right)
  - North: positive Y (forward)
  - Up: positive Z (up)
- **Camera frame**: X (right), Y (down), Z (into scene)

### Gimbal Angles
- **Pitch**: Tilt up/down (-90° = nadir/straight down)
- **Roll**: Tilt left/right (0° = level)
- **Yaw**: Rotation around vertical axis

## Performance Tips

1. **Use smaller YOLOv8 model for speed**:
   ```python
   processor = ImageProcessor("yolov8n.pt")  # nano - fastest
   # vs
   processor = ImageProcessor("yolov8x.pt")  # xlarge - more accurate
   ```

2. **Reduce image resolution for faster processing**:
   ```python
   image = cv2.imread("photo.jpg")
   image = cv2.resize(image, (2048, 1536))  # Half resolution
   ```

3. **Batch process images**:
   ```python
   for img_path in glob.glob("*.jpg"):
       # Process multiple images
   ```

## API Reference

### CoordinateTransform
```python
from coordinate_transform import CoordinateTransform

transformer = CoordinateTransform(ref_lat, ref_lon, ref_alt)

# GPS → ENU
enu = transformer.gps_to_enu(lat, lon, alt)

# ENU → GPS
lat, lon = transformer.enu_to_gps(enu)

# Drone GPS → ENU
enu = transformer.drone_to_enu(drone_lat, drone_lon, drone_alt)
```

### CameraModel
```python
from camera_model import CameraModel

camera = CameraModel(metadata)

# Pixel → normalized camera coordinates
norm = camera.pixel_to_normalized(pixel_x, pixel_y)

# Pixel → ray direction
ray = camera.pixel_to_camera_ray(pixel_x, pixel_y)

# Get specs
info = camera.get_sensor_info()
```

### GeometryEngine
```python
from geometry import GeometryEngine

geometry = GeometryEngine(
    drone_altitude_m=50,
    gimbal_pitch_deg=-90,
    gimbal_roll_deg=0,
    gimbal_yaw_deg=0,
    flight_yaw_deg=0
)

# Transform ray to world frame
world_ray = geometry.transform_camera_ray_to_world(camera_ray)

# Find ground intersection
intersection_enu = geometry.ray_ground_intersection(
    world_ray, 
    drone_position_enu, 
    ground_z=0
)
```

### ImageProcessor
```python
from image_processor import ImageProcessor

processor = ImageProcessor("yolov8n.pt")

# Load image
image = processor.load_image("photo.jpg")

# Detect objects
detections = processor.detect_objects(image, confidence=0.5)

# Calculate GSD
gsd = processor.calculate_gsd(altitude_m=50)

# Visualize
vis = processor.draw_detections(image, detections)
cv2.imshow("Detections", vis)
```

### GeoReferencer
```python
from geo_referencer import GeoReferencer, DroneState

georeferencer = GeoReferencer(ref_lat, ref_lon, ref_alt)

drone_state = DroneState(
    latitude_deg=lat,
    longitude_deg=lon,
    altitude_m=alt,
    yaw_deg=0
)

# Single detection
geo_obj = georeferencer.detect_to_gps(
    detection, drone_state,
    image_width, image_height, gsd
)

# Batch detections
geo_objects = georeferencer.batch_detect_to_gps(
    detections, drone_state,
    image_width, image_height, gsd
)

# Export
georeferencer.export_detections_csv(geo_objects, "results.csv")
georeferencer.export_detections_json(geo_objects, "results.json")
```

## Next Steps

1. **Try the examples**: `python examples.py`
2. **Run tests**: `python test_system.py`
3. **Read the full README**: Check README.md for detailed documentation
4. **Process your own images**: Use your drone photos to test the system

## Support Resources

- **Camera specs**: Check your drone's manual for focal length and sensor dimensions
- **EXIF data**: Use online tools to view embedded metadata
- **Coordinates**: Use Google Maps or similar to verify GPS results
- **Error debugging**: Enable verbose output to trace issues

## Quick Reference - Common Drone Specs

| Drone | Focal Length | Sensor Size | Notes |
|-------|-------------|-------------|-------|
| DJI Mini 2 | 3.67 mm | 6.3 × 4.72 mm | Most common |
| DJI Air 2S | 5.3 mm | 13.3 × 8.8 mm | Better zoom |
| DJI Matrice 200 | 3.6 mm | 6.17 × 4.63 mm | Professional |

Check your drone's specifications in the manual for exact values.

---

**Last Updated**: January 2025  
**Quick Start Version**: 1.0
