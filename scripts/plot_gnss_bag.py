import click
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import os
import folium
from pathlib import Path
from tqdm import tqdm


def extract_gps_from_rosbag2(db3_path: Path, topic_name: str = "/fix"):
    msg_type = get_message("sensor_msgs/msg/NavSatFix")
    gps_data = []

    conn = sqlite3.connect(db3_path)
    cursor = conn.cursor()

    # Get the topic_id from the topic name
    cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    result = cursor.fetchone()
    if result is None:
        raise ValueError(f"Topic '{topic_name}' not found in bag.")
    topic_id = result[0]

    # Fetch all messages from that topic
    cursor.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,))
    msg_count = cursor.fetchone()[0]

    cursor.execute("SELECT data FROM messages WHERE topic_id=?", (topic_id,))
    for row in tqdm(cursor.fetchall(), total=msg_count, desc="Extracting GPS data"):
        msg = deserialize_message(row[0], msg_type)
        gps_data.append((msg.latitude, msg.longitude))

    conn.close()
    return gps_data


def plot_gps_on_map(gps_points, output_file="gps_map.html"):
    if not gps_points:
        raise ValueError("No GPS points to plot.")

    # Center the map on the first point
    m = folium.Map(location=gps_points[0], zoom_start=15)

    # Optional: satellite tiles
    folium.TileLayer(
        tiles="https://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
        attr="Google Satellite",
        name="Google Satellite",
        max_zoom=20,
        subdomains=["mt0", "mt1", "mt2", "mt3"],
    ).add_to(m)

    # Plot all points
    for coord in tqdm(gps_points, total=len(gps_points), desc="Plotting GPS data"):
        folium.CircleMarker(location=coord, radius=3, color="red", fill=True).add_to(m)

    # Optionally connect them with a line
    folium.PolyLine(gps_points, color="blue", weight=2.5, opacity=0.8).add_to(m)

    # Save the map
    m.save(output_file)


@click.command()
@click.argument("bag_db_path", type=click.Path(exists=True, path_type=Path))
@click.option("--topic", default="/fix", help="Topic name with GPS data")
@click.option("--output", default="gps_map.html", help="Output HTML file")
def main(bag_db_path, topic, output):
    """Extract GPS data from a ROS2 bag and plot it on a satellite map."""
    gps_data = extract_gps_from_rosbag2(bag_db_path, topic)
    plot_gps_on_map(gps_data, output)


if __name__ == "__main__":
    main()
