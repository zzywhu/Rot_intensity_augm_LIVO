import open3d as o3d
import numpy as np
import os
import sys

def flip_xy_coordinates(input_pcd_path, output_pcd_path):
    """
    Read a PCD file, flip the x and y coordinates (multiply by -1), 
    and save the result as a new PCD file.
    
    Args:
        input_pcd_path: Path to the input PCD file
        output_pcd_path: Path to save the output PCD file
    """
    # Check if the input file exists
    if not os.path.exists(input_pcd_path):
        print(f"Error: Input file {input_pcd_path} does not exist.")
        return False
    
    # Read the input PCD file
    print(f"Reading PCD file: {input_pcd_path}")
    pcd = o3d.io.read_point_cloud(input_pcd_path)
    
    # Convert to numpy array
    points = np.asarray(pcd.points)
    
    # Flip x and y coordinates
    points[:, 0] = -points[:, 0]  # Negate x
    points[:, 1] = -points[:, 1]  # Negate y
    
    # Create a new point cloud object
    flipped_pcd = o3d.geometry.PointCloud()
    flipped_pcd.points = o3d.utility.Vector3dVector(points)
    
    # Copy colors if they exist
    if pcd.has_colors():
        flipped_pcd.colors = pcd.colors
    
    # Copy normals if they exist
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        # Flip normal directions for x and y components to match flipped coordinates
        normals[:, 0] = -normals[:, 0]
        normals[:, 1] = -normals[:, 1]
        flipped_pcd.normals = o3d.utility.Vector3dVector(normals)
    
    # Write the flipped point cloud to the output file
    print(f"Writing flipped PCD file to: {output_pcd_path}")
    o3d.io.write_point_cloud(output_pcd_path, flipped_pcd)
    
    print(f"Successfully flipped x and y coordinates and saved to {output_pcd_path}")
    return True

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python convert.py <input_pcd_file> <output_pcd_file>")
        print("Example: python convert.py input.pcd output.pcd")
    else:
        input_pcd_path = sys.argv[1]
        output_pcd_path = sys.argv[2]
        flip_xy_coordinates(input_pcd_path, output_pcd_path)
