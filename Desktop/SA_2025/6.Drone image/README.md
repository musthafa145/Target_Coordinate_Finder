# Drone Image Geo-Referencing System

A comprehensive Python system for converting pixel coordinates in drone images to real-world GPS coordinates. Integrates YOLOv8 object detection with sophisticated coordinate transformation pipelines.

## Overview

This system provides two complementary approaches for drone image analysis:

### 1. **Target Coordinate Calculator (main.py)**
Converts manual pixel coordinates to GPS using embedded EXIF metadata.

- **Input**: Drone image file (with EXIF metadata)
- **Process**: Metadata parsing → Camera modeling → Ray-ground intersection → GPS conversion
- **Output**: GPS coordinates for specified pixels
- **Accuracy**: Suitable for flat-earth scenarios (<1 km horizontal distance)

### 2. **Automated Object Detection Pipeline**
Automatically detects objects in images and geo-references them.

- **Input**: Drone image + drone state (GPS, altitude, orientation)
- **Process**: YOLOv8 detection → Pixel-to-GPS conversion → Export results
- **Output**: CSV/JSON with object GPS locations
- **Best for**: Survey missions, object tracking, spatial analysis

## System Architecture

### Core Modules

```
coordinate_transform.py     → GPS ↔ ENU (East-North-Up) conversions
    ├─ GPS to ENU
    ├─ ENU to GPS
    └─ Flat-earth approximation (valid <100 km)

camera_model.py            → Camera intrinsic parameters
    ├─ Pixel ↔ normalized coordinates
    ├─ Focal length & sensor dimensions
    └─ Intrinsic matrix computation

geometry.py                → 3D geometric operations
    ├─ Euler angle rotations
    ├─ Ray-ground intersections
    └─ Flight/gimbal attitude handling

image_processor.py         → Image analysis with YOLOv8
    ├─ Image loading & resizing
    ├─ Object detection
    ├─ GSD (Ground Sample Distance) calculation
    └─ Visualization

geo_referencer.py          → Integration layer
    ├─ Pixel detections → GPS coordinates
    ├─ Batch processing
    └─ CSV/JSON export

main.py                    → Entry point & CLI
    ├─ Metadata-based conversion
    ├─ Interactive & single-shot modes
    └─ Results display
```

## Installation

### Requirements
- Python 3.8+
- OpenCV (cv2)
- NumPy
- Pillow (PIL)
- YOLOv8 (ultralytics)
- exifread

### Setup

```bash
# Install dependencies
pip install opencv-python numpy pillow ultralytics exifread

# Verify installation
python main.py --help
```

## Usage Guide

### Method 1: Target Coordinate Calculator (Metadata-Based)

#### Interactive Mode
```bash
python main.py image.jpg
```
Then enter pixel coordinates interactively:
```
Enter pixel coordinates (x y): 1920 1080
Result GPS Coordinates:
  Latitude:  37.123456°
  Longitude: -122.456789°
```

#### Single Conversion
```bash
python main.py image.jpg 1920 1080
```

#### Flags & Options
```bash
python main.py image.jpg [pixel_x] [pixel_y] [options]
```

### Method 2: Automated Detection & Geo-Referencing

#### Basic Usage
```bash
python detect_and_georef.py \
    --image drone_image.jpg \
    --drone-lat 37.123456 \
    --drone-lon -122.456789 \
    --altitude 50.0
```

#### Advanced Options
```bash
python detect_and_georef.py \
    --image image.jpg \
    --drone-lat 37.123456 \
    --drone-lon -122.456789 \
    --altitude 50.0 \
    --yaw 45.0 \
    --ref-lat 37.120000 \
    --ref-lon -122.450000 \
    --confidence 0.6 \
    --output ./results/ \
    --model yolov8m.pt
```

#### Output Files
- `detections.jpg` - Image with detection boxes
- `detections.csv` - Tabular format (object, lat, lon, confidence, etc.)
- `detections.json` - Complete data with ENU coordinates

## Mathematical Background

### Ground Sample Distance (GSD)
```
GSD = (Altitude × Sensor_Width) / (Focal_Length × Image_Width)
```
Example: 50m altitude, 6.3mm sensor, 3.67mm focal length, 4000px width
```
GSD = (50 × 6.3) / (3.67 × 4000) ≈ 0.0215 m/pixel
```

### Coordinate Transformations
1. **Pixel → Normalized Camera Frame**
   - Compensate for focal length and principal point
   - Result: [x_norm, y_norm, z=1]

2. **Camera Frame → World Frame (ENU)**
   - Apply gimbal and flight Euler rotations
   - Z-Y-X rotation convention

3. **Ray-Ground Intersection**
   - Ray: P(t) = origin + t × direction
   - Ground plane: z = 0
   - Solve for intersection point t

4. **ENU → GPS**
   - Equirectangular approximation
   - dlat = north / EARTH_RADIUS
   - dlon = east / (EARTH_RADIUS × cos(lat))

### Coordinate Systems

**ENU (East-North-Up)**
- East: positive X (right)
- North: positive Y (forward)
- Up: positive Z (upward)
- Origin: Reference GPS point

**Camera Frame**
- X: right in image
- Y: down in image (note negation)
- Z: into scene (depth)

**World Frame (NED for drones)**
- North, East, Down
- Converts to/from ENU using sign flips

## Camera Specifications

### Default Values (DJI Matrice 200)
- Focal Length: 3.6 mm
- Sensor Width: 6.17 mm
- Sensor Height: 4.63 mm
- Image Resolution: 4000 × 3000 px

### DJI Mini 2 (Alternative)
- Focal Length: 3.67 mm
- Sensor Width: 6.3 mm
- Sensor Height: 4.72 mm
- Image Resolution: 4000 × 3000 px

**Note**: Values are overridden by EXIF metadata if present.

## Limitations & Accuracy Considerations

1. **Flat-Earth Assumption**
   - Valid for distances < 1 km horizontally
   - Beyond 1 km, use WGS84 ellipsoid models

2. **Gimbal Calibration**
   - Assumes gimbal angles accurate in metadata
   - Systematic errors if gimbal is miscalibrated

3. **Sensor Model**
   - Assumes rectilinear distortion model
   - Wide-angle lenses may need radial distortion correction

4. **Ground Assumptions**
   - Assumes ground is flat (z = 0 in ENU)
   - Hillsides/elevation changes not modeled

5. **Metadata Quality**
   - GPS coordinates precision affects final accuracy
   - Altitude estimates can drift (±5m typical)

## Example Workflow

### Scenario: Finding a target object in a drone survey

```python
from main import TargetCoordinateCalculator
from image_processor import ImageProcessor
from geo_referencer import GeoReferencer, DroneState

# Load image and metadata
calc = TargetCoordinateCalculator("survey_image.jpg")

# Get drone metadata
metadata = calc.get_image_metadata()
print(f"Drone GPS: {metadata['gps_latitude']}, {metadata['gps_longitude']}")
print(f"Altitude: {metadata['altitude']}m")

# Method A: Manual pixel-to-GPS for specific pixel
target_gps = calc.pixel_to_gps(1920, 1080)
print(f"Target at pixel (1920, 1080): {target_gps['latitude']}, {target_gps['longitude']}")

# Method B: Automated detection
processor = ImageProcessor("yolov8n.pt")
image = processor.load_image("survey_image.jpg")
detections = processor.detect_objects(image, confidence=0.6)

# Convert detections to GPS
georeferencer = GeoReferencer(
    metadata['gps_latitude'], 
    metadata['gps_longitude'],
    metadata['altitude']
)

drone_state = DroneState(
    latitude_deg=metadata['gps_latitude'],
    longitude_deg=metadata['gps_longitude'],
    altitude_m=metadata['altitude'],
    yaw_deg=metadata['flight_yaw']
)

gsd = processor.calculate_gsd(metadata['altitude'])
geo_objects = georeferencer.batch_detect_to_gps(
    detections, drone_state, 
    metadata['image_width'], metadata['image_height'], 
    gsd
)

# Export results
georeferencer.export_detections_csv(geo_objects, "detections.csv")
for obj in geo_objects:
    print(f"{obj.class_name}: {obj.gps_latitude:.6f}, {obj.gps_longitude:.6f}")
```

## Troubleshooting

### "Image file not found"
- Verify image path is correct
- Check file has read permissions
- Ensure image format is supported (JPG, PNG, etc.)

### "EXIF metadata missing"
- Image doesn't have GPS/altitude information
- Use fallback parameters or manual entry
- Older cameras may not embed metadata

### "YOLOv8 not installed"
- Install: `pip install ultralytics`
- First run may download model (500MB+)
- Use smaller model if disk space limited: `yolov8n.pt`

### GPS coordinates seem incorrect
- Verify drone altitude is accurate
- Check gimbal pitch/roll calibration
- Enable flat-earth validation for distances > 1 km
- Validate against known reference points

## Performance

- **Image Loading**: < 1s
- **YOLOv8 Detection** (nano model, 4000×3000 image): ~2-5s
- **Batch GPS Conversion** (100 objects): < 1s
- **Export to CSV/JSON**: < 100ms

## Development Notes

### Adding New Object Classes
YOLOv8 detects 80 COCO classes by default (person, car, dog, etc.). To train on custom classes:

```bash
# Use your own dataset with YOLOv8
yolo detect train data=custom.yaml model=yolov8n.pt epochs=100
```

### Fine-Tuning Camera Parameters
```python
from camera_model import CameraModel

custom_camera = CameraModel(
    metadata,
    sensor_width_mm=7.0,    # Custom sensor width
    sensor_height_mm=5.25   # Custom sensor height
)
```

### Handling Non-Nadir Views
For oblique images (camera not pointing straight down), extend `GeometryEngine`:

```python
# Modify gimbal_pitch_deg (e.g., -45° for 45° angle)
geometry = GeometryEngine(
    drone_altitude_m=50,
    gimbal_pitch_deg=-45,   # Oblique angle
    gimbal_roll_deg=0,
    gimbal_yaw_deg=0
)
```

## References

- WGS84 Ellipsoid: https://en.wikipedia.org/wiki/World_Geodetic_System
- ENU Coordinates: https://en.wikipedia.org/wiki/Topocentric_coordinate_system
- YOLOv8: https://github.com/ultralytics/ultralytics
- Camera Models: https://en.wikipedia.org/wiki/Pinhole_camera_model

## License

This project is provided as-is for educational and research purposes.

---

**Last Updated**: January 2025  
**Version**: 1.0
