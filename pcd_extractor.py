import rosbag
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from tqdm import tqdm
import os

bag_path = "/home/myungw00/ROS/rosbag/kcity/2024-10-13-10-45-20.bag"
pointcloud_topic = "/velodyne_points"
out_dir = "/home/myungw00/ROS/rosbag/asd/"


os.makedirs(out_dir, exist_ok=True)

print(f"Rosbag Path: {bag_path}")
print(f"PointCloud Topic: {pointcloud_topic}")
print(f"Output Directory: {out_dir}")

with rosbag.Bag(bag_path, "r") as bag:
    count = 0
    for topic, msg, t in bag.read_messages(topics=[pointcloud_topic]):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        if not points:
            continue
        points_np = np.array(points, dtype=np.float32)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)
        filename = os.path.join(out_dir, f"cloud_{count:04d}.pcd")
        o3d.io.write_point_cloud(filename, pcd)
        print(f"Saved {filename} ({len(points)} points, time={t.to_sec()})")
        count += 1
        
print(f"Extract {count} PCDs")


