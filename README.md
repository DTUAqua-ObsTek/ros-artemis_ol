# ros-artemis_ol
A ROS driver package to read and publish accelerometer, magnetometer and gyroscope data from the Artemis OpenLog sensor.

## OpenLog Artemis

The [SparkFun OpenLog Artemis](https://www.sparkfun.com/products/16832) is an open source data logger that comes preprogrammed to automatically log IMU, GPS, serial data, and various pressure, humidity, and distance sensors. All without writing a single line of code! OpenLog Artemis, or "OLA," automatically detects, configures, and logs Qwiic sensors. The OLA is specifically designed for users who just need to capture a lot of data to a CSV and get back to their larger project.

## Why a ROS driver?

Since the OpenLog Artemis can stream data via serial, I thought it would be nice to integrate with ROS.

## Roadmap

### Planned Support for various sensors (prioritized)

[] [MS5837 Depth / Pressure Sensor](https://www.sparkfun.com/products/retired/17709)
[] u-blox GPS Modules 

## Installation

1. Move to your ROS catkin workspace src directory.
2. Clone this repository `git clone https://github.com/DTUAqua-ObsTek/ros-artemis_ol.git artemis_ol`
3. Move out to your ROS catkin workspace directory `cd ..`
4. Install most required dependences via rosdep `rosdep install --from-paths src -i`
5. Install the incredible [AHRS](https://ahrs.readthedocs.io/en/latest/) python package `python3 -m pip install ahrs`
6. Build the artemis_ol package: `catkin build artemis_ol` (if using catkin_tools), or `catkin_make --pkg artemis_ol`

## Nodes

### artemis_driver_node
1. Connect your OpenLog Artemis to your machine via USB or serial.
2. Launch a ROS server `roscore`
3. Run the driver node `rosrun artemis_ol artemis_driver_node`

#### Parameters

`~frame_id`: The frame to publish the IMU messages on (follows [REP145](https://www.ros.org/reps/rep-0145.html) standard.)
`~linear_acceleration_stddev`: Standard deviation of the accelerometers (m/s/s).
`~angular_velocity_stddev`: Standard deviation of the gyroscopes (rad/s).
`~magnetic_field_stddev`: Standard deviation of the magnetometers (T).

#### Topics

`imu/data_raw`: Raw linear acceleration and angular velocity data and covariances. Orientation is unavailable.
`imu/mag`: Raw magnetic field data and covariance.
