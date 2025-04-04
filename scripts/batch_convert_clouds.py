#!/usr/bin/env python3
"""
Batch convert .cloud files from the records directory to PLY format for MeshLab

Usage:
    python3 batch_convert_clouds.py <input_directory> <output_directory> [--limit N] [--merge]
    
Example:
    python3 batch_convert_clouds.py ../records/20250403_153313/point_clouds/ ./converted_clouds/ --limit 10
    
    # Merge all point clouds into a single PLY file:
    python3 batch_convert_clouds.py ../records/20250403_153313/point_clouds/ ./converted_clouds/ --merge
"""

import sys
import os
import argparse
import struct
import numpy as np
from datetime import datetime
from convert_cloud_to_ply import convert_cloud_to_ply

def batch_convert(input_dir, output_dir, limit=None, merge=False):
    """
    Convert all .cloud files in a directory to PLY format
    
    Args:
        input_dir: Directory containing .cloud files
        output_dir: Directory to save PLY files
        limit: Maximum number of files to convert (None for all)
        merge: If True, merge all point clouds into a single PLY file
    """
    # Check if input directory exists
    if not os.path.exists(input_dir):
        print(f"Error: Input directory {input_dir} does not exist")
        return False
    
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory {output_dir}")
    
    # Get all .cloud files
    cloud_files = [f for f in os.listdir(input_dir) if f.endswith('.cloud')]
    cloud_files.sort()  # Sort files by name
    
    if limit is not None:
        cloud_files = cloud_files[:limit]
    
    print(f"Found {len(cloud_files)} cloud files to convert")
    
    if merge:
        # Merge all point clouds into a single PLY file
        merged_ply = os.path.join(output_dir, f"merged_pointcloud_{len(cloud_files)}_frames.ply")
        merge_cloud_files(input_dir, cloud_files, merged_ply)
        print(f"Merged point cloud saved to {merged_ply}")
    else:
        # Convert each file separately
        successful = 0
        for i, cloud_file in enumerate(cloud_files):
            input_path = os.path.join(input_dir, cloud_file)
            output_path = os.path.join(output_dir, cloud_file.replace('.cloud', '.ply'))
            
            print(f"Converting {i+1}/{len(cloud_files)}: {cloud_file}")
            result = convert_cloud_to_ply(input_path, output_path)
            if result:
                successful += 1
        
        print(f"Successfully converted {successful}/{len(cloud_files)} files to PLY format in {output_dir}")
    
    return True

def merge_cloud_files(input_dir, cloud_files, output_file):
    """
    Merge multiple .cloud files into a single PLY file
    
    Args:
        input_dir: Directory containing the cloud files
        cloud_files: List of cloud filenames
        output_file: Output PLY file path
    """
    # Collect all valid points from all files
    all_points = []
    
    # Process each cloud file
    for i, cloud_file in enumerate(cloud_files):
        input_path = os.path.join(input_dir, cloud_file)
        print(f"Reading {i+1}/{len(cloud_files)}: {cloud_file}")
        
        try:
            with open(input_path, 'rb') as f:
                # Read header (first 5 bytes should be "CLOUD")
                header = f.read(5).decode('utf-8')
                
                if header != "CLOUD":
                    print(f"Skipping invalid file: {cloud_file}")
                    continue
                
                # Skip extra byte after header
                f.read(1)
                
                # Read the rest of the file as binary data
                data = f.read()
                
                # X, Y, Z (float32) followed by RGB (uint32) - 16 bytes per point
                point_size = 16
                num_points = len(data) // point_size
                
                # Parse and filter points
                for j in range(num_points):
                    offset = j * point_size
                    
                    # Extract coordinates and color
                    x = struct.unpack('f', data[offset:offset+4])[0]
                    y = struct.unpack('f', data[offset+4:offset+8])[0]
                    z = struct.unpack('f', data[offset+8:offset+12])[0]
                    rgb_packed = struct.unpack('I', data[offset+12:offset+16])[0]
                    
                    # Unpack RGB values
                    r = (rgb_packed & 0x000000FF)
                    g = (rgb_packed & 0x0000FF00) >> 8
                    b = (rgb_packed & 0x00FF0000) >> 16
                    
                    # Filter valid points
                    if (np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and 
                        not (x == 0 and y == 0 and z == 0) and
                        abs(x) < 1000 and abs(y) < 1000 and abs(z) < 1000):
                        
                        # Add a small offset in z for each frame to visualize the sequence
                        z_offset = i * 0.05  # 5cm separation between frames
                        all_points.append((x, y, z + z_offset, r, g, b))
        
        except Exception as e:
            print(f"Error processing {cloud_file}: {str(e)}")
    
    # Write merged point cloud
    print(f"Writing merged point cloud with {len(all_points)} points")
    
    with open(output_file, 'wb') as ply:
        # Write header
        header = f"""ply
format binary_little_endian 1.0
comment Merged point cloud from {len(cloud_files)} .cloud files
comment Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
element vertex {len(all_points)}
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
"""
        ply.write(header.encode('ascii'))
        
        # Write point data
        for point in all_points:
            x, y, z, r, g, b = point
            ply.write(struct.pack('<fffBBB', x, y, z, r, g, b))
    
    print(f"Merged point cloud saved to {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Batch convert .cloud files to PLY format')
    parser.add_argument('input_dir', help='Directory containing .cloud files')
    parser.add_argument('output_dir', help='Directory to save PLY files')
    parser.add_argument('--limit', type=int, help='Maximum number of files to convert')
    parser.add_argument('--merge', action='store_true', help='Merge all point clouds into a single PLY file')
    
    args = parser.parse_args()
    
    batch_convert(args.input_dir, args.output_dir, args.limit, args.merge)

if __name__ == "__main__":
    main()