#!/usr/bin/env python3
from artemis_ol.interface import LoggerReader
import argparse
from pathlib import Path
from csv import reader
from ahrs.filters import EKF
import numpy as np
from scipy.spatial.transform import Rotation
from scipy import signal
import pandas as pd
import warnings
from matplotlib import pyplot as plt
from scipy import signal as signal
from ahrs.utils import WMM


if __name__=="__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("log_file", nargs="+", type=Path, help="Path to log file or directories containing log files.")
    ap.add_argument("-r", "--recursive", action="store_true", help="Search directories recursively for log files.")
    ap.add_argument("--plot", action="store_true", help="Flag to plot filtered data afterwards.")
    ap.add_argument("-f", "--frequency", type=float, default=100.0, help="Samping frequency.")
    ap.add_argument("-d", "--disable_mag", action="store_true", help="Flag to omit magnetometer measurements.")
    ap.add_argument("--sigma_a", type=float, default=0.07217913743449356, help="Standard deviation of accelerometer (m²/s⁴).")
    ap.add_argument("--sigma_g", type=float, default=0.0294051717887628, help="Standard deviation of gyroscope (rad²/s²)")
    ap.add_argument("--sigma_m", type=float, default=1.5530569285629352e-06, help="Standard deviation of magnetometer (T²).")
    ap.add_argument("--latlon", nargs=2, type=float, default=[55.78452765101064, 12.51507263494401], help="Latitude Longitude for WMM")
    args = ap.parse_args()
    log_files = []
    # Collect all the log files
    for log_file in args.log_file:
        if log_file.is_file():
            try:
                log_files.append(log_file.resolve(True))
            except FileNotFoundError:
                warnings.warn(f"Skipping [Not Found]: {log_file}")
                continue
        elif log_file.is_dir():
            if args.recursive:
                log_files.extend(log_file.rglob("*.csv"))
            else:
                log_files.extend(log_file.glob("*.csv"))
        else:
            warnings.warn(f"{log_file} neither a file nor a directory, skipping.")
            continue
    # Filter out the log files with _ekf_ori_est as suffix in stem (avoids recursion)
    log_files = [log_file for log_file in log_files if "_ekf_ori_est" not in log_file.name]

    # Lookup WMM dip angle
    dip = WMM(latitude=args.latlon[0], longitude=args.latlon[1]).I

    # Process each log file
    for log_file in log_files:
        print(f"Reprocessing orientation for {log_file}")
        acceleration = []
        angular_velocity = []
        magnetic_field = []
        rpy = []
        # Collect magnetometer data from LATIAS log
        with open(log_file, "r", newline="") as f:
            r = reader(f)
            for data in r:
                logger_data = ",".join(data[2:])
                parsed_data = LoggerReader.parse_log_msg(logger_data)
                if parsed_data is None:
                    continue
                rpy.append([float(d) for d in data[-15:-12]])
                acceleration.append([parsed_data[x] for x in ("ax", "ay", "az")])
                angular_velocity.append([parsed_data[x] for x in ("wx", "wy", "wz")])
                magnetic_field.append([parsed_data[x] for x in ("mx", "my", "mz")])
        # if the log was empty, skip
        if not len(acceleration):
            continue
        # Convert and calibrate acceleration
        acceleration = np.array(acceleration) / 1000.0 * 9.80865  # convert milli G to m/s/s
        acc_calib = np.array([0.997251, 0.000000, 0.000000, -0.008793,
                              0.000000, 0.996243, 0.000000, 0.111275,
                              0.000000, 0.000000, 0.993007, -0.037248,
                              0.000000, 0.000000, 0.000000, 1.000000]).reshape((4, 4))
        acceleration = np.c_[acceleration, np.ones((acceleration.shape[0], 1))]
        acceleration = acc_calib @ acceleration.T
        acceleration = (acceleration[:3, :] / acceleration[3, :]).T

        # Convert and calibrate angular velocity
        angular_velocity = np.array(angular_velocity) * np.pi / 180.0  # convert dps to rad/s
        ang_calib = np.array([1.0, 0.000000, 0.000000, 0.014525851765573208,
                              0.000000, 1.0, 0.000000, -6.702064327658279e-05,
                              0.000000, 0.000000, 1.0, -0.004786565473594447,
                              0.000000, 0.000000, 0.000000, 1.000000]).reshape((4, 4))
        angular_velocity = np.c_[angular_velocity, np.ones((angular_velocity.shape[0], 1))]
        angular_velocity = ang_calib @ angular_velocity.T
        angular_velocity = (angular_velocity[:3, :] / angular_velocity[3, :]).T

        # Convert and calibrate magnetic field
        magnetic_field = np.array(magnetic_field) * 1e-6  # convert uT to T
        mag_calib = np.array([1.0728226719, 0.0090861975, 0.0092164637, -0.0000408731,
                              0.0090861975, 1.0336617766, -0.0181571431, -0.0000473644,
                              0.0092164637, -0.0181571431, 1.0648442707, -0.0000139285,
                              0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]).reshape((4, 4))
        magnetic_field = np.c_[magnetic_field, np.ones((magnetic_field.shape[0], 1))]
        magnetic_field = mag_calib @ magnetic_field.T
        magnetic_field = (magnetic_field[:3, :] / magnetic_field[3, :]).T * 1e6  # Convert to uT

        # Filter magnetic field spikes by applying a low pass filter to the magnetometer data
        if magnetic_field.shape[0] > 18:
            lowpass = 1  # Low pass frequency in Hz
            magnetic_field = signal.filtfilt(*signal.butter(5, lowpass, btype='low', fs=args.frequency), magnetic_field, axis=0)
        else:
            warnings.warn("Could not low pass filter the magnetic field, effects of magnetometer spikes is much more pronounced.")
        
        print(f"Filtering orientation with EKF.")
        if not args.disable_mag:
            ekf = EKF(angular_velocity, acceleration, magnetic_field,
            frequency=args.frequency, var_gyr=args.sigma_g, var_acc=args.sigma_a, var_mag=args.sigma_m * 1e12, magnetic_ref=dip, frame="ENU")
        else:
            ekf = EKF(angular_velocity, acceleration, frequency=args.frequency, var_gyr=args.sigma_g, var_acc=args.sigma_a, frame="ENU")
        print(f"Filtering done.")
        quat = ekf.Q
        R = Rotation.from_quat(ekf.Q[:, [1, 2, 3, 0]])
        euler = R.as_euler("xyz", degrees=True)
        rpy = np.array(rpy)
        # rpy is in N-WU, I will not defend this...
        rpy[:, 1] = -rpy[:, 1]
        # Rotate to ENU
        R = Rotation.from_euler("xyz", (0,0,-90), degrees=True)
        rpy = R.apply(rpy)
        if args.plot:
            fig, ax = plt.subplots(4, 1)
            ax[0].plot(acceleration)
            ax[0].set_ylabel("acc (m/s/s)")
            ax[1].plot(angular_velocity)
            ax[1].set_ylabel("omega (rad/s)")
            ax[2].plot(magnetic_field)
            ax[2].set_ylabel("mag (uT)")
            colors = ["r", "g", "b"]
            for i, e in enumerate(euler.T):
                ax[3].plot(e, colors[i])
            for i, e in enumerate(rpy.T):
                ax[3].plot(e, colors[i]+"--")
            ax[3].set_ylabel("eul (deg)")
            plt.show()
        out_path = log_file.with_name(log_file.stem + "_ekf_ori_est" + log_file.suffix)
        data = pd.read_csv(log_file)
        data["Roll"], data["Pitch"], data["Yaw"] = euler.T
        data["Qw"], data["Qx"], data["Qy"], data["Qz"] = quat.T
        print(f"Writing Roll, Pitch, Yaw Data to {out_path}.")
        data.to_csv(out_path, sep=",", index=False)
