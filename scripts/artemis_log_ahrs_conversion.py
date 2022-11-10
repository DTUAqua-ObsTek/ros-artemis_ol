#!/usr/bin/env python3
from artemis_ol.interface import LoggerReader
import argparse
from pathlib import Path
from csv import reader, DictReader, writer
from ahrs.filters import EKF, Complementary
import numpy as np
from scipy.spatial.transform import Rotation
import pandas as pd
import warnings
from matplotlib import pyplot as plt
from scipy import signal as signal


if __name__=="__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("log_file", nargs="+", type=Path, help="Path to log file or directories containing log files.")
    ap.add_argument("-r", "--recursive", action="store_true", help="Search directories recursively for log files.")
    ap.add_argument("--plot", action="store_true", help="Flag to plot filtered data afterwards.")
    ap.add_argument("-f", "--frequency", type=float, default=100.0, help="Samping frequency.")
    args = ap.parse_args()
    log_files = []
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
    log_files = [log_file for log_file in log_files if "_ekf_ori_est" not in log_file.name]
    for log_file in log_files:
        print(f"Reprocessing orientation for {log_file}")
        acceleration = []
        angular_velocity = []
        magnetic_field = []
        with open(log_file, "r", newline="") as f:
            r = reader(f)
            for data in r:
                logger_data = ",".join(data[2:])
                parsed_data = LoggerReader.parse_log_msg(logger_data)
                if parsed_data is None:
                    continue
                acceleration.append([parsed_data[x] for x in ("ax", "ay", "az")])
                angular_velocity.append([parsed_data[x] for x in ("wx", "wy", "wz")])
                magnetic_field.append([parsed_data[x] for x in ("mx", "my", "mz")])
        if not len(acceleration):
            continue
        acceleration = np.array(acceleration) / 1000.0 * 9.80865  # convert milli G to m/s/s
        acc_calib = np.array([0.997251, 0.000000, 0.000000, -0.008793,
                              0.000000, 0.996243, 0.000000, 0.111275,
                              0.000000, 0.000000, 0.993007, -0.037248,
                              0.000000, 0.000000, 0.000000, 1.000000]).reshape((4, 4))
        acceleration = np.c_[acceleration, np.ones((acceleration.shape[0], 1))]
        acceleration = acc_calib @ acceleration.T
        acceleration = (acceleration[:3, :] / acceleration[3, :]).T

        angular_velocity = np.array(angular_velocity) * np.pi / 180.0  # convert dps to rad/s
        ang_calib = np.array([1.0, 0.000000, 0.000000, -0.014525851765573208,
                              0.000000, 1.0, 0.000000, 6.702064327658279e-05,
                              0.000000, 0.000000, 1.0, 0.004786565473594447,
                              0.000000, 0.000000, 0.000000, 1.000000]).reshape((4, 4))
        angular_velocity = np.c_[angular_velocity, np.ones((angular_velocity.shape[0], 1))]
        angular_velocity = ang_calib @ angular_velocity.T
        angular_velocity = (angular_velocity[:3, :] / angular_velocity[3, :]).T

        magnetic_field = np.array(magnetic_field) * 1e-6  # convert uT to T
        mag_calib = np.array([1.0728226719, 0.0090861975, 0.0092164637, -0.0000408731,
                              0.0090861975, 1.0336617766, -0.0181571431, -0.0000473644,
                              0.0092164637, -0.0181571431, 1.0648442707, -0.0000139285,
                              0.0000000000, 0.0000000000, 0.0000000000, 1.0000000000]).reshape((4, 4))
        magnetic_field = np.c_[magnetic_field, np.ones((magnetic_field.shape[0], 1))]
        magnetic_field = mag_calib @ magnetic_field.T
        magnetic_field = (magnetic_field[:3, :] / magnetic_field[3, :]).T * 1e9

        # lowpass = 0.5  # Low pass frequency in Hz
        # highpass = 0.01  # High pass frequency in Hz

        # # Low pass filter on acceleration to remove impulse force spikes
        # acceleration = signal.filtfilt(*signal.butter(5, lowpass, btype='low', fs=10), acceleration, axis=0)
        # # High pass filter to remove sensor drift (low frequency)
        # angular_velocity = signal.filtfilt(*signal.butter(5, highpass, btype='high', fs=10), angular_velocity, axis=0)
        
        print(f"Filtering orientation with EKF.")
        ekf = EKF(angular_velocity, acceleration, magnetic_field,
        frequency=args.frequency, var_gyr=0.0008646641279266516, var_acc=0.00520982788078751, var_mag=2.4119858233573378e-12)
        print(f"Filtering done.")
        quat = ekf.Q
        R = Rotation.from_quat(ekf.Q[:, [1, 2, 3, 0]])
        euler = R.as_euler("xyz", degrees=True)
        if args.plot:
            fig, ax = plt.subplots(4, 1)
            ax[0].plot(acceleration)
            ax[0].set_ylabel("acc (m/s/s)")
            ax[1].plot(angular_velocity)
            ax[1].set_ylabel("omega (rad/s)")
            ax[2].plot(magnetic_field)
            ax[2].set_ylabel("mag (nT)")
            ax[3].plot(euler)
            ax[3].set_ylabel("eul (deg)")
            plt.show()
        out_path = log_file.with_name(log_file.stem + "_ekf_ori_est" + log_file.suffix)
        data = pd.read_csv(log_file)
        data["Roll"], data["Pitch"], data["Yaw"] = euler.T
        print(f"Writing Roll, Pitch, Yaw Data to {out_path}.")
        data.to_csv(out_path, sep=",", index=False)
