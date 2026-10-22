#!/usr/bin/env python3
"""
Vectorized Helical 3D Point Cloud Reconstruction from Low-Rate Mechanical Sonar.

Unrolls helical motion distortion (motion skew) produced by mechanical profiling
sonars (e.g., Imagenex 831A) during continuous ROV motion by interpolating SE(3)
trajectory poses per individual acoustic beam using vectorized NLERP and Einstein
summation (np.einsum), followed by Open3D outlier removal and voxel downsampling.
"""

import argparse
import numpy as np
import open3d as o3d
import pandas as pd
from scipy.spatial.transform import Rotation as R
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

SENSOR_RPY = [0.0, 90.0, 0.0]


def read_poses(bag_path: str, topic_odom: str) -> pd.DataFrame:
    """Extract timestamped 6-DOF poses from a ROS 2 MCAP bag."""
    storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    reader.set_filter(StorageFilter(topics=[topic_odom]))

    times, poses = [], []
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        times.append(t_ns * 1e-9)
        poses.append([p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    if not times:
        raise ValueError(f"No odometry messages found on topic {topic_odom}")

    df = pd.DataFrame(poses, columns=["x", "y", "z", "qx", "qy", "qz", "qw"], index=times)
    return df[~df.index.duplicated(keep="first")]


def interpolate_poses_vectorized(timestamps: np.ndarray, df_poses: pd.DataFrame):
    """
    Vectorized SE(3) pose interpolation (linear position + NLERP with hemisphere
    correction on S^3 for quaternions) across arbitrary beam timestamps.
    """
    idxs = np.searchsorted(df_poses.index, timestamps)
    idxs = np.clip(idxs, 1, len(df_poses) - 1)

    t0 = df_poses.index[idxs - 1].values
    t1 = df_poses.index[idxs].values
    dt = np.where((t1 - t0) == 0, 1e-9, t1 - t0)
    ratios = ((timestamps - t0) / dt)[:, np.newaxis]

    p0 = df_poses.iloc[idxs - 1][["x", "y", "z"]].values
    p1 = df_poses.iloc[idxs][["x", "y", "z"]].values
    pos_interpolated = p0 + (p1 - p0) * ratios

    q0 = df_poses.iloc[idxs - 1][["qx", "qy", "qz", "qw"]].values
    q1 = df_poses.iloc[idxs][["qx", "qy", "qz", "qw"]].values

    # Shortest-arc hemisphere correction on S^3
    dot = np.sum(q0 * q1, axis=1, keepdims=True)
    q1 = np.where(dot < 0, -q1, q1)

    q_interp = q0 * (1.0 - ratios) + q1 * ratios
    q_final = q_interp / np.linalg.norm(q_interp, axis=1, keepdims=True)

    return pos_interpolated, q_final


def reconstruct(
    bag_path: str,
    topic_odom: str,
    topic_cloud: str,
    output_file: str,
    scan_frequency: float = 1.0,
    helical_deskew: bool = True,
) -> o3d.geometry.PointCloud:
    """Reconstruct a 3D tunnel point cloud from asynchronous odometry and sonar scans."""
    df_odom = read_poses(bag_path, topic_odom)

    storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    reader.set_filter(StorageFilter(topics=[topic_cloud]))

    global_pcd = o3d.geometry.PointCloud()
    r_sensor = R.from_euler("xyz", SENSOR_RPY, degrees=True)

    while reader.has_next():
        _, data, t_ns = reader.read_next()
        t_base = t_ns * 1e-9

        msg = deserialize_message(data, PointCloud2)
        points_list = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points_list:
            continue

        points_np = np.array(points_list)
        if points_np.dtype.names:
            points_np = np.column_stack((points_np["x"], points_np["y"], points_np["z"]))
        points_np = points_np.astype(np.float64)
        points_np = points_np[np.isfinite(points_np).all(axis=1)]
        if len(points_np) == 0:
            continue

        points_body = (r_sensor.as_matrix() @ points_np.T).T

        if helical_deskew:
            azimuths = np.arctan2(points_np[:, 1], points_np[:, 0])
            scan_phase = (azimuths + np.pi) / (2.0 * np.pi)
            point_times = t_base + scan_phase * (1.0 / scan_frequency)

            pos_interp, quat_interp = interpolate_poses_vectorized(point_times, df_odom)
            rot_matrices = R.from_quat(quat_interp).as_matrix()
            points_global = np.einsum("nij,nj->ni", rot_matrices, points_body) + pos_interp
        else:
            pos_interp, quat_interp = interpolate_poses_vectorized(np.array([t_base]), df_odom)
            r_robot = R.from_quat(quat_interp[0]).as_matrix()
            points_global = (r_robot @ points_body.T).T + pos_interp[0]

        chunk = o3d.geometry.PointCloud()
        chunk.points = o3d.utility.Vector3dVector(points_global)
        global_pcd += chunk

    global_pcd = global_pcd.remove_non_finite_points()
    global_pcd, _ = global_pcd.remove_radius_outlier(nb_points=6, radius=0.5)
    global_pcd = global_pcd.voxel_down_sample(voxel_size=0.05)
    global_pcd.estimate_normals()
    o3d.io.write_point_cloud(output_file, global_pcd)
    return global_pcd


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Helical 3D underwater sonar point cloud reconstruction.")
    parser.add_argument("--bag", required=True, help="Path to input ROS 2 MCAP bag")
    parser.add_argument("--odom-topic", default="/bluerov2/odometry/ukf", help="Odometry topic")
    parser.add_argument("--cloud-topic", default="/bluerov2/cloud", help="PointCloud2 sonar topic")
    parser.add_argument("--output", default="tunnel_reconstruction.ply", help="Output PLY/PCD path")
    parser.add_argument("--freq", type=float, default=1.0, help="Mechanical sonar sweep frequency (Hz)")
    args = parser.parse_args()

    reconstruct(args.bag, args.odom_topic, args.cloud_topic, args.output, scan_frequency=args.freq)
