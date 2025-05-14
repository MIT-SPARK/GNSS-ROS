#!/usr/bin/env python3

import sqlite3
import argparse
import utm
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extract_navsatfix_from_bag(db3_path: Path, topic_name: str = "/fix"):
    msg_type = get_message("sensor_msgs/msg/NavSatFix")
    gps_data = []

    conn = sqlite3.connect(db3_path)
    cursor = conn.cursor()

    # Get topic ID
    cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    result = cursor.fetchone()
    if result is None:
        raise ValueError(f"Topic '{topic_name}' not found in bag.")
    topic_id = result[0]

    # Fetch messages
    cursor.execute("SELECT data FROM messages WHERE topic_id=?", (topic_id,))
    for row in cursor.fetchall():
        msg = deserialize_message(row[0], msg_type)
        gps_data.append((msg.latitude, msg.longitude, msg.altitude))

    conn.close()
    return gps_data


def save_as_kitti(gps_data, filename, start_idx=None, end_idx=None):
    # Slice the data
    sliced_data = gps_data[start_idx:end_idx]

    with open(filename, "w") as f:
        for lat, lon, z in sliced_data:
            x, y, _, _ = utm.from_latlon(lat, lon)
            f.write(f"1 0 0 {x} 0 1 0 {y} 0 0 1 {z}\n")
    print(f"Saved KITTI-formatted GPS data to {filename}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert NavSatFix ROS2 bag to KITTI format"
    )
    parser.add_argument(
        "bag_path", type=Path, help="Path to ROS2 SQLite bag file (.db3)"
    )
    parser.add_argument("--topic", default="/fix", help="Topic name with GPS data")
    parser.add_argument(
        "--output", default="fix.txt", type=Path, help="Output KITTI txt file"
    )
    parser.add_argument(
        "--start", type=int, default=None, help="Start index for slicing"
    )
    parser.add_argument(
        "--end", type=int, default=None, help="End index for slicing (exclusive)"
    )
    args = parser.parse_args()

    gps_data = extract_navsatfix_from_bag(args.bag_path, args.topic)
    save_as_kitti(gps_data, args.output, args.start, args.end)


if __name__ == "__main__":
    main()
