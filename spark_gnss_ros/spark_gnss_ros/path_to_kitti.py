#!/usr/bin/env python3

import argparse
import yaml
import numpy as np


def convert_path_to_kitti(input_yaml_file, output_kitti_file):
    # Load the path from the YAML file
    with open(input_yaml_file, "r") as file:
        path = yaml.safe_load(file)

    # Open the KITTI file for writing
    with open(output_kitti_file, "w") as kitti_file:
        for pose_stamped in path["poses"]:
            position = pose_stamped["pose"]["position"]
            orientation = pose_stamped["pose"]["orientation"]

            # Convert quaternion to rotation matrix
            rotation_matrix = quaternion_to_rotation_matrix(orientation)

            # Write the transformation matrix in KITTI format (single row without [0 0 0 1])
            kitti_file.write(
                f"{rotation_matrix[0, 0]} {rotation_matrix[0, 1]} {rotation_matrix[0, 2]} {position['x']} "
                f"{rotation_matrix[1, 0]} {rotation_matrix[1, 1]} {rotation_matrix[1, 2]} {position['y']} "
                f"{rotation_matrix[2, 0]} {rotation_matrix[2, 1]} {rotation_matrix[2, 2]} {position['z']}\n"
            )
    print(f"Converted path to KITTI format and saved as {output_kitti_file}")


def quaternion_to_rotation_matrix(quat):
    # Compute the rotation matrix using quaternion-to-matrix conversion formula
    x, y, z, w = quat["x"], quat["y"], quat["z"], quat["w"]
    R = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )
    return R


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Convert YAML path to KITTI format")
    parser.add_argument("input_yaml", type=str, help="Input YAML file containing path")
    parser.add_argument(
        "output_kitti", type=str, help="Output KITTI file to save the trajectory"
    )
    args = parser.parse_args()

    # Convert the path to KITTI format
    convert_path_to_kitti(args.input_yaml, args.output_kitti)


if __name__ == "__main__":
    main()
