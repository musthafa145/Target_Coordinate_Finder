# Mathematical Reference - Drone Image Geo-Referencing

## Overview

This document provides mathematical foundations for the pixel-to-GPS conversion pipeline used in drone image analysis.

## 1. Camera Model

### 1.1 Pinhole Camera Model

The pinhole camera projects 3D world points onto a 2D image plane:

$$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \frac{1}{Z_c} \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}$$

Where:
- $(u, v)$ = image coordinates (pixels)
- $(X_c, Y_c, Z_c)$ = camera frame coordinates
- $f_x, f_y$ = focal length in pixels
- $(c_x, c_y)$ = principal point (image center)

### 1.2 Focal Length Calculation

From physical parameters:

$$f_x = \frac{f_{mm} \cdot \text{pixel density}_x}{\text{mm}}$$

$$f_y = \frac{f_{mm} \cdot \text{pixel density}_y}{\text{mm}}$$

Where focal length $f_{mm}$ is in millimeters.

**Pixel density** (pixels per mm):

$$\text{density}_x = \frac{\text{image width (px)}}{\text{sensor width (mm)}}$$

### 1.3 Pixel to Normalized Camera Coordinates

Convert pixel coordinates to normalized (focal length = 1):

$$x_{norm} = \frac{u - c_x}{f_x}$$

$$y_{norm} = \frac{-(v - c_y)}{f_y}$$

Note: $y$ is negated because image $v$ increases downward.

### 1.4 Pixel to Ray Direction

The ray direction (unit vector) from pixel coordinates:

$$\mathbf{r}_{camera} = \frac{1}{\|\cdot\|} \begin{bmatrix} x_{norm} \\ y_{norm} \\ 1 \end{bmatrix}$$

Where $\|\cdot\|$ denotes vector normalization: $\|\mathbf{v}\| = \sqrt{v_x^2 + v_y^2 + v_z^2}$

## 2. Coordinate Frame Transformations

### 2.1 Euler Angles (Z-Y-X Convention)

Rotation matrices applied in order: Yaw (Z) → Pitch (Y) → Roll (X)

**Z-rotation (Yaw, $\psi$)**:

$$R_Z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**Y-rotation (Pitch, $\theta$)**:

$$R_Y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

**X-rotation (Roll, $\phi$)**:

$$R_X(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$

**Combined transformation**:

$$R_{world} = R_Z(\psi) \cdot R_Y(\theta) \cdot R_X(\phi)$$

### 2.2 Camera Ray to World Frame

Transform ray from camera frame to world (ENU) frame:

$$\mathbf{r}_{world} = R_{world} \cdot \mathbf{r}_{camera}$$

This accounts for both gimbal orientation and flight attitude.

## 3. Ray-Ground Intersection

### 3.1 Ray Equation

A ray in parametric form starting from point $\mathbf{P}_0$ in direction $\mathbf{d}$:

$$\mathbf{P}(t) = \mathbf{P}_0 + t \cdot \mathbf{d}, \quad t \geq 0$$

Where:
- $\mathbf{P}_0$ = drone position in ENU = $(0, 0, h)$ for altitude $h$
- $\mathbf{d}$ = normalized ray direction
- $t$ = distance along ray (scalar parameter)

### 3.2 Ground Plane Intersection

Ground plane at altitude $z = 0$ (flat-earth assumption).

Solve for $t$ where ray meets ground:

$$P_0^z + t \cdot d^z = 0$$

$$t = -\frac{P_0^z}{d^z} = -\frac{h}{d^z}$$

Where $h = $ drone altitude, $d^z = $ ray's Z-component.

**Conditions for valid intersection**:
- $|d^z| > \epsilon$ (ray not parallel to ground)
- $t > 0$ (intersection in front of camera)

### 3.3 Intersection Point (ENU)

$$\mathbf{P}_{ground} = \mathbf{P}_0 + t \cdot \mathbf{d}$$

$$\begin{bmatrix} E \\ N \\ U \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ h \end{bmatrix} + t \begin{bmatrix} d_E \\ d_N \\ d_U \end{bmatrix}$$

Result: $\mathbf{P}_{ground} = (t \cdot d_E, t \cdot d_N, 0)$

## 4. Coordinate System Conversions

### 4.1 ENU to GPS (Equirectangular Approximation)

Convert ENU offset to GPS (valid for distances < 1 km):

$$\Delta\phi = \frac{N}{R}$$

$$\Delta\lambda = \frac{E}{R \cos\phi_{ref}}$$

$$\phi = \phi_{ref} + \Delta\phi$$

$$\lambda = \lambda_{ref} + \Delta\lambda$$

Where:
- $\phi$ = latitude (radians)
- $\lambda$ = longitude (radians)
- $R$ = Earth radius ≈ 6,371,000 m
- $(E, N)$ = ENU offset (meters)
- Subscript $_{ref}$ = reference point

### 4.2 GPS to ENU

$$E = R \cdot \Delta\lambda \cdot \cos\phi_{ref}$$

$$N = R \cdot \Delta\phi$$

$$U = h - h_{ref}$$

Where $\Delta\phi = \phi - \phi_{ref}$ and $\Delta\lambda = \lambda - \lambda_{ref}$

## 5. Ground Sample Distance (GSD)

### 5.1 GSD Definition

GSD is the size of one pixel as projected on the ground.

$$\text{GSD} = \frac{H \cdot W_{sensor}}{f \cdot W_{image}}$$

Where:
- $H$ = drone altitude (meters)
- $W_{sensor}$ = sensor physical width (mm)
- $f$ = focal length (mm)
- $W_{image}$ = image width (pixels)

### 5.2 Pixel Size on Ground

Distance on ground corresponding to $n$ pixels:

$$\text{Distance}_{meters} = n \times \text{GSD}$$

### 5.3 Image Coverage at Ground

Total ground area covered by the image:

$$\text{Width}_{ground} = W_{image} \times \text{GSD}$$

$$\text{Height}_{ground} = H_{image} \times \text{GSD}$$

## 6. Pixel to ENU for Nadir View

### 6.1 Simplified Conversion (Camera Straight Down)

For nadir view (gimbal pitch = -90°), pixel location maps to ENU directly:

$$E = \left(u - \frac{W_{image}}{2}\right) \times \text{GSD}$$

$$N = -\left(v - \frac{H_{image}}{2}\right) \times \text{GSD}$$

Where $(u, v)$ are pixel coordinates.

Note the negative sign for $N$ because image $v$ increases downward.

### 6.2 With Drone Yaw Rotation

If drone has yaw angle $\psi$:

$$\begin{bmatrix} E' \\ N' \end{bmatrix} = \begin{bmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{bmatrix} \begin{bmatrix} E \\ N \end{bmatrix}$$

## 7. Error Sources and Bounds

### 7.1 Flat-Earth Approximation Error

For distance $d$ on Earth surface:

$$\text{Error} \approx \frac{d^3}{24 R^2}$$

At 1 km: error ≈ 0.08 m (negligible)  
At 10 km: error ≈ 50 m (significant)

### 7.2 Gimbal Calibration Error

Angular error $\delta\theta$ at distance $d$:

$$\text{Position Error} \approx d \cdot \tan(\delta\theta)$$

Example: 50° gimbal error at 1 km → ~10 m error

### 7.3 GPS Accuracy

Typical DJI drone GPS accuracy: ±3-5 meters

### 7.4 Altitude Error

Typical DJI altitude estimation error: ±1-3% of altitude

## 8. Example Calculation

### Scenario
- Drone at 50 m altitude over flat ground
- DJI Mini 2 camera: focal length 3.67 mm, sensor 6.3 × 4.72 mm
- Image: 4000 × 3000 pixels
- Pixel of interest: (1920, 1080) - slightly right of center

### Step 1: Calculate GSD

$$\text{GSD} = \frac{50 \times 6.3}{3.67 \times 4000} = 0.0215 \text{ m/pixel}$$

### Step 2: Pixel to Normalized Coordinates

Intrinsic matrix:
$$f_x = \frac{3.67 \times 4000}{6.3} = 2333 \text{ px}$$
$$f_y = \frac{3.67 \times 3000}{4.72} = 2335 \text{ px}$$
$$c_x = 2000, c_y = 1500$$

For pixel (1920, 1080):
$$x_{norm} = \frac{1920 - 2000}{2333} = -0.0343$$
$$y_{norm} = -\frac{1080 - 1500}{2335} = 0.1798$$

### Step 3: Camera Ray (Normalized)

$$\mathbf{r}_{camera} = \frac{1}{\sqrt{0.0343^2 + 0.1798^2 + 1^2}} \begin{bmatrix} -0.0343 \\ 0.1798 \\ 1 \end{bmatrix}$$

$$\approx \begin{bmatrix} -0.0343 \\ 0.1796 \\ 0.9983 \end{bmatrix}$$

### Step 4: World Ray (Nadir, No Rotation)

For nadir view: $R_{world} = I$ (identity)

$$\mathbf{r}_{world} = \begin{bmatrix} -0.0343 \\ 0.1796 \\ 0.9983 \end{bmatrix}$$

### Step 5: Ground Intersection

$$t = -\frac{50}{0.9983} = -50.09$$

Wait, this should be positive. For nadir view with camera pointing down, the ray should have negative z-component:

$$\mathbf{r}_{world} = \begin{bmatrix} -0.0343 \\ 0.1796 \\ -0.9983 \end{bmatrix}$$ (corrected)

$$t = -\frac{50}{-0.9983} = 50.09$$

Ground intersection:
$$E = -0.0343 \times 50.09 = -1.72 \text{ m}$$
$$N = 0.1796 \times 50.09 = 8.99 \text{ m}$$

### Step 6: Convert to GPS

Using reference GPS and ENU offset:

$$\Delta\phi = \frac{8.99}{6,371,000} \text{ rad} = 1.41 \times 10^{-6} \text{ rad} = 0.291" \text{ (arcsec)}$$

$$\Delta\lambda = \frac{-1.72}{6,371,000 \times \cos\phi_{ref}} \text{ rad}$$

Convert to degrees and add to reference coordinates to get final GPS.

## 9. Key Assumptions

1. **Flat Earth**: Valid for $d < 1$ km horizontal distance
2. **Constant altitude**: Ground is at sea level (z = 0)
3. **Negligible atmospheric refraction**: Valid for typical drones
4. **Rectilinear distortion model**: No radial/tangential distortion
5. **Accurate metadata**: GPS, gimbal angles, altitude from drone are accurate

## 10. Coordinate System Diagram

```
              North (Y)
                 ↑
                 |
   West ←--------+--------→ East (X)
                 |
              Camera
             (Gimbal)
              ↓ (Down)
           Ground
              ↓ Up (Z)

ENU Frame:
- E (East): +X, perpendicular to meridian
- N (North): +Y, along meridian
- U (Up): +Z, radially outward from Earth

Camera Frame:
- X: right in image
- Y: down in image
- Z: into scene (depth)
```

## References

- Hartley, R., & Zisserman, A. (2003). Multiple View Geometry in Computer Vision. Cambridge University Press.
- Snyder, J. P. (1987). Map Projections - A Working Manual. USGS Professional Paper 1395.
- DJI Technical Documentation: https://www.dji.com/
- WGS84 Coordinate System: https://en.wikipedia.org/wiki/World_Geodetic_System

---

**Mathematical Reference Version**: 1.0  
**Last Updated**: January 2025
