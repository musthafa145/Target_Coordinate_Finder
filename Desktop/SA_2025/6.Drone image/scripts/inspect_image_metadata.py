#!/usr/bin/env python3
"""
Extract and summarize EXIF/XMP metadata from drone images.

Usage:
    python inspect_image_metadata.py -o output.csv                 # CSV from current dir
    python inspect_image_metadata.py -p image.jpg                  # Single image inspection
    python inspect_image_metadata.py -d /path/to/folder -o out.csv # CSV from folder
"""

import argparse
import sys
import csv
from pathlib import Path
from PIL import Image
import xml.etree.ElementTree as ET


def extract_numeric_metadata(img, img_path):
    """
    Extract numeric EXIF and XMP metadata from image.
    
    Returns:
        dict: Numeric metadata fields
    """
    meta = {
        'File': str(img_path),
        'DateTime': None,
        'GPSLatitude': None,
        'GPSLongitude': None,
        'GPSAltitude': None,
        'AbsoluteAltitude': None,
        'RelativeAltitude': None,
        'GimbalRollDegree': None,
        'GimbalYawDegree': None,
        'GimbalPitchDegree': None,
        'FlightRollDegree': None,
        'FlightPitchDegree': None,
        'FlightYawDegree': None,
        'FocalLength_mm': None,
    }
    
    exif = img.getexif()
    
    if exif:
        # DateTime (tag 306)
        if 306 in exif:
            meta['DateTime'] = exif[306]
        
        # Focal Length (tag 37386 or in Exif sub-IFD)
        if 37386 in exif:
            f = exif[37386]
            meta['FocalLength_mm'] = float(f.numerator) / float(f.denominator)
        elif 34665 in exif:
            try:
                exif_sub = exif.get_ifd(34665)
                if 37386 in exif_sub:
                    f = exif_sub[37386]
                    meta['FocalLength_mm'] = float(f.numerator) / float(f.denominator)
            except:
                pass
        
        # GPS from IFD 34853
        try:
            gps_ifd = exif.get_ifd(34853)
            if gps_ifd:
                # GPS Latitude (tag 2)
                if 2 in gps_ifd:
                    lat_tuple = gps_ifd[2]
                    lat_deg = float(lat_tuple[0].numerator) / float(lat_tuple[0].denominator)
                    lat_min = float(lat_tuple[1].numerator) / float(lat_tuple[1].denominator)
                    lat_sec = float(lat_tuple[2].numerator) / float(lat_tuple[2].denominator)
                    lat = lat_deg + lat_min/60 + lat_sec/3600
                    
                    if 1 in gps_ifd and gps_ifd[1] == 'S':
                        lat = -lat
                    meta['GPSLatitude'] = lat
                
                # GPS Longitude (tag 4)
                if 4 in gps_ifd:
                    lon_tuple = gps_ifd[4]
                    lon_deg = float(lon_tuple[0].numerator) / float(lon_tuple[0].denominator)
                    lon_min = float(lon_tuple[1].numerator) / float(lon_tuple[1].denominator)
                    lon_sec = float(lon_tuple[2].numerator) / float(lon_tuple[2].denominator)
                    lon = lon_deg + lon_min/60 + lon_sec/3600
                    
                    if 3 in gps_ifd and gps_ifd[3] == 'W':
                        lon = -lon
                    meta['GPSLongitude'] = lon
                
                # GPS Altitude (tag 6)
                if 6 in gps_ifd:
                    alt = gps_ifd[6]
                    meta['GPSAltitude'] = float(alt.numerator) / float(alt.denominator)
        except:
            pass
    
    # Try to extract XMP fields
    if hasattr(img, 'info') and 'XMP' in img.info:
        try:
            xmp_bytes = img.info['XMP']
            if isinstance(xmp_bytes, bytes):
                xmp_str = xmp_bytes.decode('utf-8')
            else:
                xmp_str = xmp_bytes
            
            root = ET.fromstring(xmp_str)
            
            # Define XMP namespaces
            namespaces = {
                'drone-dji': 'http://www.dji.com/drone-dji/1.0/',
                'rdf': 'http://www.w3.org/1999/02/22-rdf-syntax-ns#',
                'xmp': 'http://ns.adobe.com/xap/1.0/'
            }
            
            # Extract numeric fields from XMP
            xmp_fields = [
                'AbsoluteAltitude', 'RelativeAltitude',
                'GimbalRollDegree', 'GimbalPitchDegree', 'GimbalYawDegree',
                'FlightRollDegree', 'FlightPitchDegree', 'FlightYawDegree'
            ]
            
            for field in xmp_fields:
                for ns_prefix, ns_uri in namespaces.items():
                    xpath = f'.//{{{ns_uri}}}{field}'
                    elem = root.find(xpath)
                    if elem is not None and elem.text:
                        try:
                            meta[field] = float(elem.text)
                            break
                        except:
                            pass
        except Exception as e:
            pass
    
    return meta


def inspect_single_image(img_path):
    """Print detailed inspection of a single image."""
    print(f"\n{'='*80}")
    print(f"IMAGE INSPECTION: {img_path}")
    print(f"{'='*80}\n")
    
    try:
        img = Image.open(img_path)
        print(f"✅ Image opened successfully")
        print(f"   Size: {img.width} x {img.height}")
        print(f"   Format: {img.format}")
        print(f"   Mode: {img.mode}\n")
    except Exception as e:
        print(f"❌ Failed to open image: {e}")
        return
    
    # Extract and display all EXIF
    print("EXIF DATA:")
    print("-" * 80)
    exif = img.getexif()
    if exif:
        from PIL.Image import Exif
        from PIL.ExifTags import TAGS
        
        for tag_id, value in exif.items():
            tag_name = TAGS.get(tag_id, tag_id)
            print(f"  {tag_id:5d} ({tag_name:40s}): {str(value)[:70]}")
    else:
        print("  (No EXIF data)")
    
    # Extract GPS IFD details
    if exif and 34853 in exif:
        print("\nGPS IFD (Tag 34853) - Detailed:")
        print("-" * 80)
        try:
            gps_ifd = exif.get_ifd(34853)
            for tag_id, value in gps_ifd.items():
                print(f"  Tag {tag_id}: {value}")
        except Exception as e:
            print(f"  Error reading GPS IFD: {e}")
    
    # Extract numeric metadata
    print("\nNUMERIC METADATA (Extracted):")
    print("-" * 80)
    meta = extract_numeric_metadata(img, img_path)
    for key, value in meta.items():
        if value is not None:
            if isinstance(value, float):
                print(f"  {key:30s}: {value:.10g}")
            else:
                print(f"  {key:30s}: {value}")
    
    # XMP extraction attempt
    print("\nXMP DATA:")
    print("-" * 80)
    if hasattr(img, 'info') and 'XMP' in img.info:
        print("  (XMP data found in image info)")
    else:
        print("  (No XMP data found)")
    
    print(f"\n{'='*80}\n")


def inspect_all_in_dir(root_dir, output_csv=None):
    """
    Walk directory tree, extract metadata from all images, optionally save to CSV.
    
    Args:
        root_dir (Path): Root directory to search
        output_csv (str): Optional output CSV filename
    """
    root_dir = Path(root_dir)
    
    if not root_dir.exists():
        print(f"❌ Directory not found: {root_dir}")
        return
    
    # Find all images
    image_extensions = {'.jpg', '.jpeg', '.png', '.tiff', '.bmp'}
    images = []
    
    for ext in image_extensions:
        images.extend(root_dir.glob(f'**/*{ext}'))
        images.extend(root_dir.glob(f'**/*{ext.upper()}'))
    
    print(f"📷 Found {len(images)} images in {root_dir}")
    
    if not images:
        print("⚠️  No images found")
        return
    
    # Extract metadata from each image
    metadata_list = []
    
    for i, img_path in enumerate(sorted(images), 1):
        try:
            print(f"  [{i}/{len(images)}] Processing: {img_path.name}...", end='', flush=True)
            img = Image.open(img_path)
            meta = extract_numeric_metadata(img, img_path)
            metadata_list.append(meta)
            print(" ✅")
        except Exception as e:
            print(f" ❌ ({e})")
    
    # Save to CSV if requested
    if output_csv:
        output_path = Path(output_csv)
        try:
            with open(output_path, 'w', newline='', encoding='utf-8') as f:
                if metadata_list:
                    fieldnames = list(metadata_list[0].keys())
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(metadata_list)
            
            print(f"\n✅ Metadata saved to: {output_path}")
            print(f"   {len(metadata_list)} images processed")
        except Exception as e:
            print(f"\n❌ Failed to write CSV: {e}")
    
    # Print summary to console
    print(f"\n{'='*80}")
    print("METADATA SUMMARY")
    print(f"{'='*80}\n")
    
    for meta in metadata_list:
        print(f"File: {Path(meta['File']).name}")
        if meta['DateTime']:
            print(f"  DateTime: {meta['DateTime']}")
        if meta['GPSLatitude'] and meta['GPSLongitude']:
            print(f"  GPS: ({meta['GPSLatitude']:.6f}, {meta['GPSLongitude']:.6f})")
        if meta['GPSAltitude']:
            print(f"  Altitude (GPS): {meta['GPSAltitude']:.2f}m")
        if meta['RelativeAltitude']:
            print(f"  Altitude (Relative): {meta['RelativeAltitude']:.2f}m")
        print()


def main():
    """Parse arguments and run inspection."""
    parser = argparse.ArgumentParser(
        description='Extract EXIF/XMP numeric metadata from drone images',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLES:
  # Generate CSV from current directory images
  %(prog)s -o metadata.csv

  # Generate CSV from specific folder
  %(prog)s -d ./photos -o metadata.csv

  # Inspect single image in detail
  %(prog)s -p image.jpg

  # Combine inspection and CSV (inspect current dir + save CSV)
  %(prog)s -o metadata.csv
        """
    )
    
    parser.add_argument('-d', '--directory', default='.',
                       help='Directory to search for images (default: current dir)')
    parser.add_argument('-p', '--photo', help='Path to single image for detailed inspection')
    parser.add_argument('-o', '--output', help='Output CSV filename')
    
    args = parser.parse_args()
    
    if args.photo:
        inspect_single_image(args.photo)
    else:
        inspect_all_in_dir(args.directory, args.output)


if __name__ == '__main__':
    main()
