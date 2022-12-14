#!/usr/bin/env python3


import serial
import threading
import argparse
from serial.tools import list_ports
from typing import Union, Tuple
from collections import deque
import logging
import datetime as dt
import time
import contextlib
import re

LOOPTIME_MS = 10e-6
YEARMONTHDAY = re.compile("^\d{4}/\d{2}/\d{2}$")
DAYMONTHYEAR = re.compile("^\d{2}/\d{2}/\d{4}$")


def find_logger(vendor_id: Union[str, int], product_id: Union[str, int]) -> Union[
    None, serial.tools.list_ports_linux.SysFS]:
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


@contextlib.contextmanager
def nonblocking(lock):
    locked = lock.acquire(False)
    try:
        yield locked
    finally:
        if locked:
            lock.release()


class LoggerManager(serial.Serial):
    def __init__(self, vid: str = "0x1a86", pid: str = "0x7523", port: str = None, baudrate: int = 115200,
                 timeout: float = 1.0, queue_size: int = None, log: bool = False):
        serial.Serial.__init__(self)  # init the serial object
        self._serial_lock = threading.Lock()  # init the serial lock
        self._headers = None  # We don't know what data the artemis logger is providing
        self._log_flag = log  # Set to send logs
        self._run_flag = False  # Flag to run the read/write threads
        self._in_buffer = deque(maxlen=queue_size)  # Buffer for received data from artemis
        self._out_buffer = deque(maxlen=queue_size)  # buffer to send data to artemis
        self._pid = pid  # product id of artemis
        self._vid = vid  # vendor id of artemis
        # Set the port of the serial
        if port is None:
            self.port = self._get_port()  # try to find the vid:pid on the USB bus
        else:
            self.port = port
        self.baudrate = baudrate  # set the baudrate
        self.timeout = timeout  # set the timeout (used for readline method)
        self._disconnect_flag = True  # flag to indicate if the serial is current disconnected
        self.serial_connect()  # Connect to the serial device
        self._read_thread = threading.Thread(target=self._read_serial)  # Read thread
        self._write_thread = threading.Thread(target=self._write_serial)  # Write thread

    def _get_port(self) -> str:
        device = None
        if self._log_flag:
            logging.info(f"Waiting for OL Artemis device connection {self._vid}:{self._pid}.")
        while device is None:
            device = find_logger(self._vid, self._pid)
            time.sleep(LOOPTIME_MS)
        logging.info(f"Connected to OL Artemis device.")
        return device.device

    def serial_connect(self):
        while self._disconnect_flag:
            try:
                if self._serial_lock.acquire(False):
                    if self.isOpen():
                        self.close()
                    self.port = self._get_port()
                    self.open()
                    self._disconnect_flag = False
            except serial.SerialException:
                time.sleep(LOOPTIME_MS)
            finally:
                self._serial_lock.release()

    def config(self):
        time.sleep(5)
        self.send("\n")
        time.sleep(5)
        self.send("h\n")
        time.sleep(5)

    def start(self):
        if self._run_flag:
            if self._log_flag:
                logging.warning("Manager already started.")
        else:
            if self._log_flag:
                logging.info("Starting manager.")
            self._run_flag = True
            self._read_thread.start()
            self._write_thread.start()
            self.config()

    def stop(self):
        if not self._run_flag:
            if self._log_flag:
                logging.warning("Manager already stopped.")
        else:
            if self._log_flag:
                logging.info("Stopping manager.")
            self._run_flag = False

    def join(self):
        self._read_thread.join()
        self._write_thread.join()
        self.close()

    @staticmethod
    def parse_headers(msg: str) -> Union[dict, tuple]:
        if msg.lower().startswith("rtcdate"):
            return dict(headers=tuple([m.lower() for m in msg.split(",") if len(m)]))
        return tuple([m.lower() for m in msg.split(",") if len(m)])

    @staticmethod
    def parse_data(raw_data: Tuple[str], headers: Tuple[str, None]) -> Union[dict, None]:
        imu_keys = (
        "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "rawax", "raway", "rawaz", "rawgx", "rawgy", "rawgz",
        "rawmx", "rawmy", "rawmz", "depth_m", "mbar", "alt_m", "degc")
        data = {header.lower(): packet for packet, header in zip(raw_data, headers)}
        try:
            date = data["rtcdate"]
            rtctime = data["rtctime"]
            ms_ = data.get("micros")
            h, m, s = rtctime.split(":")
            s, ms = s.split(".")
            ms = int(ms)
            # TODO: For now, micros and the rtctime centiseconds are not synced
            if ms_ is not None:
                ms = int(ms * 1e4 + int(ms_[-4:]))
            datestring = "{}T{}{}{}.{:<06d}".format(date, h, m, s, ms)
            if YEARMONTHDAY.match(date):
                timestamp = dt.datetime.strptime(datestring, "%Y/%m/%dT%H%M%S.%f")
            elif DAYMONTHYEAR.match(date):
                timestamp = dt.datetime.strptime(datestring, "%d/%m/%YT%H%M%S.%f")
            else:
                raise ValueError(f"Invalid date {date}.")
            imu_raw = {key: float(data.get(key)) for key in imu_keys if data.get(key) is not None}
            imu_raw.update({"timestamp": timestamp})
            return imu_raw
        except KeyError as e:
            raise KeyError(f"Logger Manager.parse_data {e}, {data}")
        except ValueError as e:
            raise ValueError(f"Logger Manager.parse_data {e}, {data}")
        except AttributeError as e:
            raise AttributeError(f"Logger Manager.parse_data {e}, {data}")

    def _read_serial(self):
        # Thread spin
        while self._run_flag:
            msg = None
            parsed = None
            # acquire access to serial object
            if self._serial_lock.acquire(False):
                try:
                    line_bytes = self.readline()  # Read a line of data
                    line = line_bytes.decode(encoding="utf-8").rstrip()  # Convert to a string
                    parsed = LoggerManager.parse_headers(line)  # Check for headers
                    # Update the headers variable if a header line has been detected
                    if isinstance(parsed, dict):
                        if self._log_flag:
                            logging.info(f"Update to headers: {parsed['headers']}")
                        self._headers = parsed["headers"]
                    # Otherwise the data is probably a log update
                    elif self._headers is not None:
                        data = LoggerManager.parse_data(parsed, self._headers)
                        if data is not None:
                            self._in_buffer.append(data)
                # If there was an issue with the serial device (eg. disconnect)
                except serial.SerialException:
                    # If the disconnect flag has not been raised
                    if not self._disconnect_flag:
                        self._disconnect_flag = True  # Raise the flag
                        self._serial_lock.release()  # Release access to the serial
                        self.serial_connect()  # Try reconnecting to the serial
                        time.sleep(LOOPTIME_MS)
                        continue  # Try again
                except TimeoutError:
                    msg = "Serial Timeout."
                except UnicodeDecodeError:
                    msg = "Decode error"
                except ValueError as e:
                    msg = f"Value parse error. {e} {parsed} {self._headers}"
                except KeyError as e:
                    msg = f"Key error. {e} {parsed}, {self._headers}"
                finally:
                    if self._log_flag:
                        if msg is not None:
                            logging.debug(msg)
                self._serial_lock.release()
            time.sleep(LOOPTIME_MS)

    def _write_serial(self):
        # Outer thread spin
        while self._run_flag:
            # Try lock access to serial
            if self._serial_lock.acquire(False):
                # If there is data to send
                if len(self._out_buffer):
                    packet = self._out_buffer.pop()
                    if isinstance(packet, str):
                        packet = packet.encode("utf-8")
                    # Try writing data to serial
                    try:
                        self.write(packet)
                    except serial.SerialException:
                        # If the disconnect flag has not been raised
                        if not self._disconnect_flag:
                            self._disconnect_flag = True
                            self._serial_lock.release()
                            self.serial_connect()
                            time.sleep(LOOPTIME_MS)
                            continue
                self._serial_lock.release()
            time.sleep(LOOPTIME_MS)

    def send(self, in_bytes: Union[bytes, str]):
        self._out_buffer.append(in_bytes)

    def __next__(self) -> Union[None, dict, int]:
        if self._disconnect_flag:
            return -1
        if len(self._in_buffer):
            return self._in_buffer.pop()


class LoggerReader(threading.Thread, serial.Serial):
    def __init__(self, vid: str = "0x1a86", pid: str = "0x7523", port: str = None, baudrate: int = 115200,
                 timeout: float = 1.0, queue_size: int = None, log: bool = False):
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
        if YEARMONTHDAY.match(date):
            timestamp = dt.datetime.strptime(datestring, "%Y/%m/%dT%H%M%S.%f")
        else:
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
    ap.add_argument("-p", "--port", type=str, default=None,
                    help="Logger device path. Use if program cannot find the vendor:product id.")
    ap.add_argument("-b", "--baudrate", type=int, default=115200, help="Logger device baudrate.")
    ap.add_argument("-l", "--logger_output", type=str, default="off", choices=["off", "screen", "log"],
                    help="Choose to log messages to screen or file.")
    ap.add_argument("--log_level", type=str, default="info", choices=["debug", "info", "warn", "error", "fatal"],
                    help="Logger level.")
    args = ap.parse_args()
    loglevel = getattr(logging, args.log_level.upper())
    filename = dt.datetime.utcnow().strftime("%y%m%dT%H%M%S.%f_artemis_ol.log")
    if args.logger_output == "log":
        logging.basicConfig(filename=filename, level=loglevel,
                            format='[%(asctime)s.%(msecs)03d %(levelname)s]\t%(message)s', datefmt="%y%m%dT%H%M%S")
    else:
        logging.basicConfig(level=loglevel,
                            format='[%(asctime)s.%(msecs)03d %(levelname)s]\t%(message)s', datefmt="%y%m%dT%H%M%S")
    log_flag = args.logger_output != "off"
    try:
        # sh = LoggerReader(args.vid, args.pid, args.port, args.baudrate, log=log_flag, queue_size=30)
        sh = LoggerManager(args.vid, args.pid, args.port, args.baudrate, log=log_flag, queue_size=30)
        sh.start()
        log_last = time.perf_counter()
        counter = 0
        while True:
            if (time.perf_counter() - log_last) > 1:
                if log_flag:
                    logging.info(f"Average Rate: {counter} Hz")
                counter = 0
                log_last = time.perf_counter()
            time.sleep(LOOPTIME_MS)
            data = next(sh)
            if data is not None:
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


if __name__ == "__main__":
    main()
