"""
Minimal DJIMetadataParser
Attempts to read basic EXIF metadata (GPS, altitude, gimbal angles) from an image.
Falls back to sensible defaults when tags are missing.
"""
from PIL import Image
from typing import Dict
import os

try:
    import exifread
except Exception:
    exifread = None


class DJIMetadataParser:
    def __init__(self, image_path: str):
        if not os.path.exists(image_path):
            raise FileNotFoundError(image_path)
        self.image_path = image_path
        self._metadata = None

    def _read_exif(self) -> Dict:
        meta = {}
        try:
            if exifread:
                with open(self.image_path, 'rb') as f:
                    tags = exifread.process_file(f, details=False)
                # Try common GPS tags
                gps_lat = tags.get('GPS GPSLatitude')
                gps_lat_ref = tags.get('GPS GPSLatitudeRef')
                gps_lon = tags.get('GPS GPSLongitude')
                gps_lon_ref = tags.get('GPS GPSLongitudeRef')
                gps_alt = tags.get('GPS GPSAltitude')
                if gps_lat and gps_lon:
                    meta['gps_latitude'] = self._rational_to_decimal(gps_lat, gps_lat_ref)
                    meta['gps_longitude'] = self._rational_to_decimal(gps_lon, gps_lon_ref)
                if gps_alt:
                    try:
                        meta['altitude'] = float(str(gps_alt))
                    except Exception:
                        meta['altitude'] = None
            # fallthrough
        except Exception:
            pass

        # Always include image dimensions
        try:
            with Image.open(self.image_path) as img:
                meta['image_width'], meta['image_height'] = img.size
        except Exception:
            meta['image_width'], meta['image_height'] = 4000, 3000

        # Provide placeholders for gimbal/flight angles and focal length
        meta.setdefault('gimbal_pitch', -90.0)
        meta.setdefault('gimbal_roll', 0.0)
        meta.setdefault('gimbal_yaw', 0.0)
        meta.setdefault('flight_yaw', 0.0)
        meta.setdefault('focal_length_mm', 3.6)
        meta.setdefault('altitude', 50.0)

        # If GPS missing, set to 0,0 (user can override)
        meta.setdefault('gps_latitude', 0.0)
        meta.setdefault('gps_longitude', 0.0)

        return meta

    def _rational_to_decimal(self, value, ref=None):
        try:
            # exifread returns Ratio objects; convert to floats
            parts = [float(x.num) / float(x.den) for x in value.values]
            deg, minutes, seconds = parts
            dec = deg + minutes / 60.0 + seconds / 3600.0
            if ref and str(ref).strip().upper() in ['S', 'W']:
                dec = -dec
            return dec
        except Exception:
            try:
                return float(str(value))
            except Exception:
                return 0.0

    def get_all_metadata(self) -> Dict:
        if self._metadata is None:
            self._metadata = self._read_exif()
        return self._metadata.copy()
