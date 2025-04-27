import branca.colormap as cm
import numpy as np
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
        cov_matrix = np.array(msg.position_covariance).reshape(3, 3)
        cov_scalar = np.trace(cov_matrix[:3, :3]) # ignore altitude
        gps_data.append((msg.latitude, msg.longitude, cov_scalar))

    conn.close()
    return gps_data


def plot_gps_on_map(gps_data, output_file="gps_map.html"):
    # Coords and covariances
    coords = [(lat, lon) for lat, lon, _ in gps_data]
    covariances = np.array([cov for _, _, cov in gps_data])

    # Normalize covariances to 0-1
    norm_cov = (covariances - covariances.min()) / (covariances.ptp() + 1e-6)

    # Create colormap: green → yellow → red
    colormap = cm.linear.YlOrRd_09.scale(0, 1)

    # Center the map on the first point
    m = folium.Map(location=coords[0], zoom_start=15)

    folium.TileLayer(
        tiles="https://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
        attr="Google Satellite",
        name="Google Satellite",
        max_zoom=20,
        subdomains=["mt0", "mt1", "mt2", "mt3"],
    ).add_to(m)

    # Plot all points
    for i, coord in tqdm(
        enumerate(coords), total=len(gps_data), desc="Plotting GPS data"
    ):
        color = colormap(norm_cov[i])
        folium.CircleMarker(location=coord, radius=3, color=color, fill=True).add_to(m)

    colormap.caption = "GPS Covariance (Uncertainty)"
    colormap.add_to(m)

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
