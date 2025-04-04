#!/usr/bin/env python3
"""
Convert .cloud files from the records directory to PLY format for MeshLab

Usage:
    python3 convert_cloud_to_ply.py <path_to_cloud_file> <output_ply_file>
    
Example:
    python3 convert_cloud_to_ply.py ../records/20250403_153313/point_clouds/1743694393802279126.cloud output.ply
"""

import sys
import os
import struct
import numpy as np
from datetime import datetime

def convert_cloud_to_ply(cloud_file, ply_file):
    """
    Convert a .cloud file to PLY format
    
    Args:
        cloud_file: Path to the .cloud file
        ply_file: Path for the output PLY file
    """
    # Check if input file exists
    if not os.path.exists(cloud_file):
        print(f"Error: Input file {cloud_file} does not exist")
        return False
    
    # Read the binary cloud file
    try:
        with open(cloud_file, 'rb') as f:
            # Read header (first 5 bytes should be "CLOUD")
            header = f.read(5).decode('utf-8')
            
            if header != "CLOUD":
                print(f"Error: Not a valid cloud file (header is {header} not CLOUD)")
                return False
                
            # Skip extra byte after header
            f.read(1)
            
            # Read the rest of the file as binary data
            data = f.read()
            
            # Assuming the format is:
            # X, Y, Z (float32) followed by RGB (uint32) - 16 bytes per point
            point_size = 16  # 4 bytes per float * 3 + 4 bytes for RGB
            
            # Calculate number of points
            num_points = len(data) // point_size
            print(f"Number of points: {num_points}")
            
            # Parse the points and filter out invalid ones
            valid_points = []
            
            for i in range(num_points):
                offset = i * point_size
                
                # Extract X, Y, Z coordinates (3 float32 values)
                x = struct.unpack('f', data[offset:offset+4])[0]
                y = struct.unpack('f', data[offset+4:offset+8])[0]
                z = struct.unpack('f', data[offset+8:offset+12])[0]
                
                # Extract RGB value (1 uint32 value)
                rgb_packed = struct.unpack('I', data[offset+12:offset+16])[0]
                
                # Unpack RGB values
                r = (rgb_packed & 0x000000FF)
                g = (rgb_packed & 0x0000FF00) >> 8
                b = (rgb_packed & 0x00FF0000) >> 16
                
                # Skip NaN, infinity and zero points
                if (np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and 
                    not (x == 0 and y == 0 and z == 0) and
                    abs(x) < 1000 and abs(y) < 1000 and abs(z) < 1000):  # Filter out extreme values
                    valid_points.append((x, y, z, r, g, b))
            
            print(f"Found {len(valid_points)} valid points out of {num_points}")
            
            if len(valid_points) == 0:
                print("No valid points found, aborting conversion")
                return False
            
            # Use binary PLY format for better compatibility with MeshLab
            with open(ply_file, 'wb') as ply:
                # Write header
                header = f"""ply
format binary_little_endian 1.0
comment Converted from .cloud file
comment Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
comment Source file: {os.path.basename(cloud_file)}
element vertex {len(valid_points)}
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
"""
                ply.write(header.encode('ascii'))
                
                # Write point data in binary
                for point in valid_points:
                    x, y, z, r, g, b = point
                    ply.write(struct.pack('<fffBBB', x, y, z, r, g, b))
            
            print(f"Successfully converted to binary PLY format with {len(valid_points)} valid points")
            print(f"Output saved to {ply_file}")
            return True
            
    except Exception as e:
        print(f"Error converting file: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 convert_cloud_to_ply.py <path_to_cloud_file> <output_ply_file>")
        return
    
    cloud_file = sys.argv[1]
    ply_file = sys.argv[2]
    
    convert_cloud_to_ply(cloud_file, ply_file)

if __name__ == "__main__":
    main()