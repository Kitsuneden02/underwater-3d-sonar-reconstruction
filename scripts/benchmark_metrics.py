#!/usr/bin/env python3
"""
State Estimation Benchmark Evaluator (EKF vs. UKF) for Underwater ROV Trajectories.

Computes 2D/3D Root Mean Square Error (RMSE), Mean Absolute Error (MAE), Maximum
Absolute Error, and Normalized Estimation Error Squared (NEES) against ground-truth
trajectories from ROS 2 bag recordings.
"""

import os
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def extract_frequency(bag_path: str, topic_name: str) -> float:
    """Compute average publication frequency (Hz) for a given ROS 2 topic."""
    times = []
    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == topic_name]
        if not connections:
            return 0.0
        for _, timestamp, _ in reader.messages(connections=connections):
            times.append(timestamp / 1e9)
    if len(times) < 2:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0])


def extract_odometry(bag_path: str, topic_name: str, is_filter: bool = False) -> pd.DataFrame:
    """Extract 3D trajectory and planar 2x2 covariance blocks from Odometry messages."""
    times, x, y, z = [], [], [], []
    c00, c01, c10, c11 = [], [], [], []

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == topic_name]
        if not connections:
            return pd.DataFrame()
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            times.append(timestamp / 1e9)
            pose = msg.pose.pose.position
            x.append(pose.x)
            y.append(pose.y)
            z.append(pose.z)
            if is_filter:
                cov = msg.pose.covariance
                c00.append(cov[0])
                c01.append(cov[1])
                c10.append(cov[6])
                c11.append(cov[7])

    data = {"time": times, "x": x, "y": y, "z": z}
    if is_filter:
        data.update({"c00": c00, "c01": c01, "c10": c10, "c11": c11})
    return pd.DataFrame(data).drop_duplicates(subset=["time"]).sort_values(by="time").reset_index(drop=True)


def calculate_advanced_metrics(df_gt: pd.DataFrame, df_filter: pd.DataFrame):
    """Synchronize timestamps and compute RMSE, MAE, Max Error, and NEES."""
    if df_gt.empty or df_filter.empty:
        return [None] * 5

    f_x = interp1d(df_filter["time"], df_filter["x"], bounds_error=False, fill_value="extrapolate")
    f_y = interp1d(df_filter["time"], df_filter["y"], bounds_error=False, fill_value="extrapolate")
    f_z = interp1d(df_filter["time"], df_filter["z"], bounds_error=False, fill_value="extrapolate")

    err_x = f_x(df_gt["time"]) - df_gt["x"]
    err_y = f_y(df_gt["time"]) - df_gt["y"]
    err_z = f_z(df_gt["time"]) - df_gt["z"]
    err_2d = np.sqrt(err_x**2 + err_y**2)

    rmse_2d = float(np.sqrt(np.nanmean(err_2d**2)))
    rmse_z = float(np.sqrt(np.nanmean(err_z**2)))
    mae_2d = float(np.nanmean(err_2d))
    max_err_2d = float(np.nanmax(err_2d))

    f_c00 = interp1d(df_filter["time"], df_filter["c00"], bounds_error=False, fill_value="extrapolate")
    f_c01 = interp1d(df_filter["time"], df_filter["c01"], bounds_error=False, fill_value="extrapolate")
    f_c10 = interp1d(df_filter["time"], df_filter["c10"], bounds_error=False, fill_value="extrapolate")
    f_c11 = interp1d(df_filter["time"], df_filter["c11"], bounds_error=False, fill_value="extrapolate")

    c00, c01 = f_c00(df_gt["time"]), f_c01(df_gt["time"])
    c10, c11 = f_c10(df_gt["time"]), f_c11(df_gt["time"])

    det = (c00 * c11) - (c01 * c10)
    det[det == 0] = 1e-12
    inv00, inv11 = c11 / det, c00 / det
    inv01, inv10 = -c01 / det, -c10 / det

    nees = err_x * (inv00 * err_x + inv01 * err_y) + err_y * (inv10 * err_x + inv11 * err_y)
    return rmse_2d, mae_2d, max_err_2d, rmse_z, float(np.nanmean(nees))
