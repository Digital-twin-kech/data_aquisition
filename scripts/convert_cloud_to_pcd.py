#!/usr/bin/env python3
"""
Convert .cloud files from the records directory to PCD format for PCL

Usage:
    python3 convert_cloud_to_pcd.py <path_to_cloud_file> <output_pcd_file>
    
Example:
    python3 convert_cloud_to_pcd.py ../records/20250403_153313/point_clouds/1743694393802279126.cloud output.pcd
"""

import sys
import os
import struct
import numpy as np
from datetime import datetime

def convert_cloud_to_pcd(cloud_file, pcd_file):
    """
    Convert a .cloud file to PCD format
    
    Args:
        cloud_file: Path to the .cloud file
        pcd_file: Path for the output PCD file
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
                    
                    # Pack RGB in PCL format (float value)
                    rgb_pcl = (r << 16) | (g << 8) | b
                    rgb_float = struct.unpack('f', struct.pack('I', rgb_pcl))[0]
                    
                    valid_points.append((x, y, z, rgb_float))
            
            print(f"Found {len(valid_points)} valid points out of {num_points}")
            
            if len(valid_points) == 0:
                print("No valid points found, aborting conversion")
                return False
            
            # Write as PCD file
            with open(pcd_file, 'w') as pcd:
                # PCD header (ASCII format)
                pcd.write("# .PCD v0.7 - Point Cloud Data file format\n")
                pcd.write(f"VERSION 0.7\n")
                pcd.write(f"FIELDS x y z rgb\n")
                pcd.write(f"SIZE 4 4 4 4\n")
                pcd.write(f"TYPE F F F F\n")
                pcd.write(f"COUNT 1 1 1 1\n")
                pcd.write(f"WIDTH {len(valid_points)}\n")
                pcd.write(f"HEIGHT 1\n")
                pcd.write(f"VIEWPOINT 0 0 0 1 0 0 0\n")
                pcd.write(f"POINTS {len(valid_points)}\n")
                pcd.write(f"DATA ascii\n")
                
                # Write point data
                for point in valid_points:
                    x, y, z, rgb = point
                    pcd.write(f"{x} {y} {z} {rgb}\n")
            
            print(f"Successfully converted to PCD format with {len(valid_points)} valid points")
            print(f"Output saved to {pcd_file}")
            return True
            
    except Exception as e:
        print(f"Error converting file: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 convert_cloud_to_pcd.py <path_to_cloud_file> <output_pcd_file>")
        return
    
    cloud_file = sys.argv[1]
    pcd_file = sys.argv[2]
    
    convert_cloud_to_pcd(cloud_file, pcd_file)

if __name__ == "__main__":
    main()