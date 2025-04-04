#!/usr/bin/env python3

"""
Generate a ChArUco board for calibration.

This script generates a ChArUco board image that can be printed for calibration.
The default board size is 5x7 squares with 6x6 ArUco markers.
"""

import argparse
import cv2
import numpy as np
import os
from datetime import datetime

def generate_charuco_board(output_path, squares_x=5, squares_y=7, 
                          square_length=200, marker_length=100,
                          dpi=300, margin_mm=10):
    """
    Generate a ChArUco board and save it as an image.
    
    Args:
        output_path: Path to save the output image
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction
        square_length: Length of each square in pixels
        marker_length: Length of each marker in pixels
        dpi: DPI for printing
        margin_mm: Margin in mm around the board
    """
    try:
        # Try using the newer OpenCV 4.x API
        # Create a dictionary with appropriate size
        try:
            # OpenCV 4.7+
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            
            # Create a ChArUco board
            board = cv2.aruco.CharucoBoard_create(
                squares_x, squares_y,
                float(square_length)/float(marker_length),
                float(marker_length)/float(square_length), 
                aruco_dict
            )
        except (AttributeError, ImportError):
            # OpenCV 4.x but different API
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            
            # Create a ChArUco board
            board = cv2.aruco.CharucoBoard(
                (squares_x, squares_y),
                float(square_length)/float(marker_length),
                float(marker_length)/float(square_length), 
                aruco_dict
            )
        
        # Calculate the image size in pixels
        margin_px = int(margin_mm * dpi / 25.4)  # Convert mm to pixels at given DPI
        img_width = squares_x * square_length + 2 * margin_px
        img_height = squares_y * square_length + 2 * margin_px
        
        # Create the image
        img = board.draw((img_width, img_height))
        
    except Exception as e:
        print(f"Warning: Error using ArUco API: {str(e)}")
        print("Creating manual ChArUco board as fallback")
        
        # Manual creation of a ChArUco board
        margin_px = int(margin_mm * dpi / 25.4)  # Convert mm to pixels at given DPI
        img_width = squares_x * square_length + 2 * margin_px
        img_height = squares_y * square_length + 2 * margin_px
        
        # Create white image
        img = np.ones((img_height, img_width), dtype=np.uint8) * 255
        
        # Draw the chessboard pattern
        square_size = square_length
        for i in range(squares_y):
            for j in range(squares_x):
                if (i + j) % 2 == 0:
                    # Draw black square
                    x1 = j * square_size + margin_px
                    y1 = i * square_size + margin_px
                    x2 = (j + 1) * square_size + margin_px
                    y2 = (i + 1) * square_size + margin_px
                    cv2.rectangle(img, (x1, y1), (x2, y2), 0, -1)
        
        # Since we can't draw real ArUco markers, we'll add a notice
        font = cv2.FONT_HERSHEY_SIMPLEX
        warning_text = "Warning: Generated checkerboard pattern without ArUco markers"
        cv2.putText(img, warning_text, (margin_px, img_height - 40), font, 0.7, 0, 2)
    
    # Add text with dimensions
    font = cv2.FONT_HERSHEY_SIMPLEX
    board_width_mm = squares_x * square_length * 25.4 / dpi
    board_height_mm = squares_y * square_length * 25.4 / dpi
    square_size_mm = square_length * 25.4 / dpi
    marker_size_mm = marker_length * 25.4 / dpi
    
    text = f"ChArUco Board: {squares_x}x{squares_y} squares, Square: {square_size_mm:.1f}mm, Marker: {marker_size_mm:.1f}mm"
    text_size = cv2.getTextSize(text, font, 1, 2)[0]
    text_x = (img_width - text_size[0]) // 2
    cv2.putText(img, text, (text_x, img_height - 20), font, 1, (0, 0, 0), 2)
    
    # Save the image
    print(f"Saving ChArUco board to {output_path}")
    print(f"Board dimensions: {board_width_mm:.1f}mm x {board_height_mm:.1f}mm at {dpi} DPI")
    print(f"Square size: {square_size_mm:.1f}mm, Marker size: {marker_size_mm:.1f}mm")
    
    cv2.imwrite(output_path, img)
    
    # Save additional metadata
    metadata_path = os.path.splitext(output_path)[0] + "_metadata.txt"
    with open(metadata_path, "w") as f:
        f.write(f"ChArUco Board Configuration\n")
        f.write(f"---------------------------\n")
        f.write(f"Generated on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Squares X: {squares_x}\n")
        f.write(f"Squares Y: {squares_y}\n")
        f.write(f"Square Size (mm): {square_size_mm:.2f}\n")
        f.write(f"Marker Size (mm): {marker_size_mm:.2f}\n")
        f.write(f"Board Dimensions: {board_width_mm:.2f}mm x {board_height_mm:.2f}mm\n")
        f.write(f"DPI: {dpi}\n")
        f.write(f"Dictionary: DICT_6X6_250\n")
        f.write(f"\nWhen printing, ensure that scaling is set to 100% (no scaling).\n")
        f.write(f"Verify the square size after printing with a ruler to confirm dimensions.\n")
    
    print(f"Calibration metadata saved to {metadata_path}")
    
    return img

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Generate a ChArUco board for calibration')
    parser.add_argument('--output', type=str, default='charuco_board.png',
                       help='Output image path')
    parser.add_argument('--squares-x', type=int, default=5,
                       help='Number of squares in X direction')
    parser.add_argument('--squares-y', type=int, default=7,
                       help='Number of squares in Y direction')
    parser.add_argument('--square-length', type=int, default=200,
                       help='Length of each square in pixels')
    parser.add_argument('--marker-length', type=int, default=120,
                       help='Length of each marker in pixels')
    parser.add_argument('--dpi', type=int, default=300,
                       help='DPI for printing')
    parser.add_argument('--margin', type=int, default=10,
                       help='Margin in mm')
    
    args = parser.parse_args()
    
    # Create the output directory if it doesn't exist
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Generate the board
    generate_charuco_board(args.output, args.squares_x, args.squares_y,
                          args.square_length, args.marker_length,
                          args.dpi, args.margin)
    
    print(f"ChArUco board generated and saved to {args.output}")
    print("Print this board on a rigid surface and use it for calibration.")

if __name__ == '__main__':
    main()