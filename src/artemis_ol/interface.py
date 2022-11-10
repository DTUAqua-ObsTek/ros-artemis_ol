#!/usr/bin/env python3


import serial
import threading
import argparse
from serial.tools import list_ports
from typing import List, Union
from collections import deque
import logging
import datetime as dt
import time


def find_logger(vendor_id: Union[str, int], product_id: Union[str, int]) ->  Union[None, serial.tools.list_ports_linux.SysFS]:
    if not isinstance(vendor_id, (int, str)):
        raise ValueError("vendor_id must be a hex string or an integer.")
    if not isinstance(product_id, (int, str)):
        raise ValueError("product_id must be a hex string or an integer.")
    for port in list_ports.comports():
        vendor_flag = False
        product_flag = False
        if port is None:
            continue
        if port.pid is None or port.vid is None:
            continue
        if isinstance(vendor_id, str):
            vendor_flag = hex(port.vid) == vendor_id
        elif isinstance(vendor_id, int):
            vendor_flag = port.vid == vendor_id
        if isinstance(product_id, str):
            product_flag = hex(port.pid) == product_id
        elif isinstance(product_id, int):
            product_flag = port.pid == product_id
        if vendor_flag and product_flag:
            return port


class LoggerReader(threading.Thread, serial.Serial):
    def __init__(self, vid: str="0x1a86", pid: str="0x7523", port: str=None, baudrate: int=115200, timeout: float=1.0, queue_size: int=None, log: bool=False):
        if port is None:
            device = find_logger(vid, pid)
            if device is None:
                if log:
                    logging.error(f"Device with {vid}:{pid} address not found.")
                raise FileNotFoundError(f"Device with {vid}:{pid} address not found.")
            port = device.device
        self._lock = threading.Lock()
        self._log_flag = log
        self._run_flag = True
        self._buffer = deque(maxlen=queue_size)
        threading.Thread.__init__(self)
        serial.Serial.__init__(self, port, baudrate, timeout=timeout)

    @staticmethod
    def parse_log_msg(raw_data: str) -> Union[None, dict]:
        data = raw_data.split(",")
        try:
            imu_data = [float(v) for v in data[2:11]]
            date, time = data[:2]
            h, m, s = time.split(":")
            s, ms = s.split(".")
            datestring = "{}T{}{}{}.{:<06d}".format(date, h, m, s, int(ms))
        except ValueError as e:
            return
        timestamp = dt.datetime.strptime(datestring, "%d/%m/%YT%H%M%S.%f")
        return dict(ax=imu_data[0], ay=imu_data[1], az=imu_data[2],
        wx=imu_data[3], wy=imu_data[4], wz=imu_data[5],
        mx=imu_data[6], my=-imu_data[7], mz=-imu_data[8], ts=timestamp)

    def stop(self):
        if self._run_flag:
            self._run_flag = False
            return
        if self._log_flag:
            logging.warning("Thread already stopped.")

    def run(self):
        while self._run_flag:
            try:
                line = self.readline().decode().rstrip()
            except UnicodeDecodeError:
                if self._log_flag:
                    logging.warning("Decode error on serial.")
                continue
            data = self.parse_log_msg(line)
            with self._lock:
                self._buffer.append(data)
            if self._log_flag:
                logging.info(data)
        self.close()

    def __next__(self) -> Union[None, dict]:
        with self._lock:
            if len(self._buffer):
                return self._buffer.pop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vid", type=str, default="0x1a86", help="Logger vendor id.")
    ap.add_argument("--pid", type=str, default="0x7523", help="Logger product id.")
    ap.add_argument("-p", "--port", type=str, default=None, help="Logger device path. Use if program cannot find the vendor:product id.")
    ap.add_argument("-b", "--baudrate", type=int, default=115200, help="Logger device baudrate.")
    ap.add_argument("-l", "--logger_output", type=str, default="off", choices=["off", "screen", "log"], help="Choose to log messages to screen or file.")
    ap.add_argument("--log_level", type=str, default="info", choices=["debug", "info", "warn", "error", "fatal"], help="Logger level.")
    args = ap.parse_args()
    loglevel = getattr(logging, args.log_level.upper())
    filename = dt.datetime.utcnow().strftime("%y%m%dT%H%M%S.%f_artemis_ol.log")
    if args.logger_output == "log":
        logging.basicConfig(filename=filename, level=loglevel,
        format='[%(asctime)s.%(msecs)06d %(levelname)s]\t%(message)s', datefmt="%y%m%dT%H%M%S")
    else:
        logging.basicConfig(level=loglevel,
        format='[%(asctime)s.%(msecs)06d %(levelname)s]\t%(message)s', datefmt="%y%m%dT%H%M%S")
    log_flag = args.logger_output != "off"
    time_window = deque(maxlen=100)
    try:
        sh = LoggerReader(args.vid, args.pid, args.port, args.baudrate, log=log_flag, queue_size=30)
        sh.start()
        log_last = time.perf_counter()
        counter = 0
        while True:
            if (time.perf_counter() - log_last) > 1:
                print(counter)
                counter = 0
                log_last = time.perf_counter()
            time.sleep(0.0000001)
            if next(sh) is not None:
                counter = counter + 1
    except KeyboardInterrupt:
        if log_flag:
            logging.info("Ctrl + C pressed.")
        sh.stop()
        sh.join()
    finally:
        logging.shutdown()
    if log_flag:
        logging.info("Finished!")


if __name__=="__main__":
    main()