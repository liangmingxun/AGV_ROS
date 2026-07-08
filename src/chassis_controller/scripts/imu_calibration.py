#!/usr/bin/env python3
import numpy as np
import rosbag
from sensor_msgs.msg import Imu
from scipy.optimize import least_squares
from tqdm import tqdm
import matplotlib.pyplot as plt
from itertools import combinations
from numpy.linalg import norm
import time
from collections import defaultdict

def plot_imu_data(time_data, acc_data, gyro_data):
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))

    axis_labels = ['x', 'y', 'z']
    colors = ['r', 'g', 'b']

    for i in range(3):
        ax.plot(time_data, acc_data[:, i], label=f'acc_{axis_labels[i]}', color=colors[i], linestyle='-')
        ax.plot(time_data, gyro_data[:, i], label=f'gyro_{axis_labels[i]}', color=colors[i], linestyle='--')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("IMU Values")
    ax.set_title("IMU Acceleration and Gyroscope Data")
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()

def is_stable_segment(accs, gyros, acc_thresh=0.1, gyro_thresh=0.1):
    # print(np.std(accs, axis=0))
    # print(np.std(gyros, axis=0))
    # print(np.all(np.std(accs, axis=0) < acc_thresh) and np.all(np.std(gyros, axis=0) < gyro_thresh))
    return np.all(np.std(accs, axis=0) < acc_thresh) and np.all(np.std(gyros, axis=0) < gyro_thresh)

def extract_static_segments(bag_path, imu_topic="/chassis/imu/data_raw", mag_topic="/chassis/imu/mag", min_duration=10, max_duration=60 * 5, step=1.0/500, top_n=20):
    bag = rosbag.Bag(bag_path)
    acc_data, gyro_data, time_data = [], [], []

    for _, msg, t in tqdm(bag.read_messages(topics=[imu_topic]), desc="Reading IMU"):
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        acc_data.append(acc)
        gyro_data.append(gyro)
        time_data.append(t.to_sec())

    acc_data = np.array(acc_data)
    gyro_data = np.array(gyro_data)
    time_data = np.array(time_data)

    mag_data = []

    for _, msg, t in tqdm(bag.read_messages(topics=[mag_topic]), desc="Reading MAG"):
        mag = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        mag_data.append(mag)

    mag_data = np.array(mag_data)

    bag.close()
    
    # plot_imu_data(time_data - time_data[0], acc_data, gyro_data)
    # exit()

    static_segments = []
    start_idx = None
    counter = 0

    window_size = int(min_duration / step)
    for i in range(window_size, len(acc_data)):
        acc_window = acc_data[i-window_size:i]
        gyro_window = gyro_data[i-window_size:i]
        if is_stable_segment(acc_window, gyro_window) and i != len(acc_data) - 1:
            if start_idx is None:
                start_idx = i - window_size
            counter += 1
        else:
            if start_idx is not None and counter >= window_size:
                end_idx = i
                duration = (end_idx - start_idx) * step
                if duration <= max_duration:
                    seg = {
                        'acc': acc_data[start_idx:end_idx].mean(axis=0),
                        'acc_std': acc_data[start_idx:end_idx].std(axis=0),
                        'gyro': gyro_data[start_idx:end_idx].mean(axis=0),
                        'mag': mag_data[start_idx:end_idx].mean(axis=0),
                        't_start': time_data[start_idx],
                        't_end': time_data[end_idx - 1]
                    }
                    static_segments.append(seg)
                start_idx = None
                counter = 0
        
        # sleep 1s
        # time.sleep(1.0)
    
    for seg in static_segments:
        print(f"static segment: {seg['t_start'] - time_data[0]:.3f}s ~ {seg['t_end'] - time_data[0]:.3f}s, duration: {(seg['t_end'] - seg['t_start']):.3f}s")

    if len(static_segments) < 6:
        raise RuntimeError(f"Not enough static segments detected ({len(static_segments)} detected).")

    def classify_face(acc_vec):
        """
        将单位加速度向量归类到 ±x / ±y / ±z 六个主方向
        """
        axis = np.argmax(np.abs(acc_vec))
        sign = np.sign(acc_vec[axis])
        labels = ['+x', '-x', '+y', '-y', '+z', '-z']
        return labels[2 * axis + (0 if sign > 0 else 1)]

    # 按稳定程度筛选前 top_n 段
    top_n = min(top_n, len(static_segments))
    static_segments.sort(key=lambda x: norm(x['acc_std']))
    selected = static_segments[:top_n]

    # 分类
    face_groups = defaultdict(list)
    for seg in selected:
        acc_dir = seg['acc'] / norm(seg['acc'])
        face = classify_face(acc_dir)
        face_groups[face].append(seg)

    # 从每个面中选择持续时间最长的一段
    chosen_faces = []
    for face, group in face_groups.items():
        best = max(group, key=lambda x: x['t_end'] - x['t_start'])
        chosen_faces.append(best)

    if len(chosen_faces) < 6:
        raise RuntimeError(f"Only {len(chosen_faces)} unique faces found, expected 6.")

    # 按照固定顺序排列（可选）
    face_order = ['+x', '-x', '+y', '-y', '+z', '-z']
    best_segments = [max(face_groups[f], key=lambda x: x['t_end'] - x['t_start']) for f in face_order if f in face_groups]

    for seg in best_segments:
        print(f"Chosen static segment: {seg['t_start'] - time_data[0]:.3f}s ~ {seg['t_end'] - time_data[0]:.3f}s, duration: {(seg['t_end'] - seg['t_start']):.3f}s")

    acc_static = np.array([s['acc'] for s in best_segments])
    gyro_static = np.array([s['gyro'] for s in best_segments])
    mag_static = np.array([s['mag'] for s in best_segments])
    return acc_static, gyro_static, mag_static

def residual_accel(params, acc_samples, g=9.81):
    b, s = np.array(params[:3]), np.array(params[3:])
    res = []
    for acc in acc_samples:
        acc_calib = (acc - b) / s
        res.append(np.linalg.norm(acc_calib) - g)
    return res

def estimate_accel_lsq(acc_samples):
    init = np.hstack([np.mean(acc_samples, axis=0), np.ones(3)])
    result = least_squares(residual_accel, init, args=(acc_samples,))
    return result.x[:3], result.x[3:]

def estimate_gyro_bias(gyro_samples):
    return np.mean(gyro_samples, axis=0)

def residual_magnetometer(params, m_raw):
    b = np.array(params[:3])
    scale = np.array(params[3:])
    residuals = []
    for m in m_raw:
        m_calib = (m - b) / scale
        residuals.append(np.linalg.norm(m_calib) - 1.0)  # 单位球拟合
    return residuals

def estimate_mag_lsq(m_raw):
    init_bias = np.mean(m_raw, axis=0)
    init_scale = np.ones(3)
    init_params = np.hstack([init_bias, init_scale])
    result = least_squares(residual_magnetometer, init_params, args=(m_raw,))
    return result.x[:3], result.x[3:]

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("bagfile", help="Input bag file")
    parser.add_argument("--imu_topic", default="/chassis/imu/data_raw", help="IMU topic name")
    parser.add_argument("--mag_topic", default="/chassis/imu/mag", help="Magnetometer topic name")
    args = parser.parse_args()

    acc_static, gyro_static, mag_static = extract_static_segments(args.bagfile, args.imu_topic, args.mag_topic)
    acc_bias, acc_scale = estimate_accel_lsq(acc_static)
    gyro_bias = estimate_gyro_bias(gyro_static)
    mag_bias, mag_scale = estimate_mag_lsq(mag_static)

    print("acc_bias:", np.array2string(acc_bias, separator=', '))
    print("acc_scale:", np.array2string(acc_scale, separator=', '))
    print("gyro_bias:", np.array2string(gyro_bias, separator=', '))
    print("mag_bias:", np.array2string(mag_bias, separator=', '))
    print("mag_scale:", np.array2string(mag_scale, separator=', '))

    # visualize
    acc_calib = (acc_static - acc_bias) / acc_scale
    acc_norm = np.linalg.norm(acc_calib, axis=1)
    plt.hist(acc_norm, bins=20)
    plt.title("Norm of Calibrated Acceleration (~9.81 m/s^2)")
    plt.grid(True)
    plt.show()