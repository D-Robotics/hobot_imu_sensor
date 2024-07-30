 Getting Started with IMU Sensor Node
---
#Intro
imu_sensor package is used for publishing ROS2 topic
 of sensor_msgs::msg::imu which contains the angular_velocity, linear_acceleration and fine-tuned
 timestamp of the robot.
In this doc, users can learn how to compile and use imu_sensor package in very detailed.
---

# Build

## Dependency

dependency libraries：
ros2 package：
- sensor_msgs
- rclcpp

## Developing Environment

- Language：C++
- Platform：X3
- Operating System：Ubuntu 20.04
- Compiling Toolchain：Linux GCC 9.3.0/Linaro GCC 9.3.0
## package Description

After package imu_sensor is compilied completely，the directory of 
config and launch as well as .so can be found in directory of
install/lib/imu_sensor or install/share/imu_sensor.

## Compiling
Either of ways of compiling on X3 in Ubuntu or PC in docker is supported.
 
### Compiling on X3 in Ubuntu
1、verify your compiling environment
- Ubuntu is running freely your X3
- source your TogetheROS bash file `source $TogetheROS_PATH/setup.bash` where
the $TogetheROS_PATH depends on your install directory
- colcon is installed already, if not :`pip install -U colcon-common-extensions`

2、compiling：
  `colcon build --packages-select imu_sensor`。


### Cross compiling on PC in docker 

1、verify your compiling environment
- Please refer docker environment deployment documentary
 (https://github.com/D-Robotics/robot_dev_config/blob/develop/README.md)
  for deploying your cross compiling environment.


2、compiling

- compiling command： 

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
  
  colcon build --packages-select mipi_cam \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
     
  ```
  

# Usage
## X3 Ubuntu

Launched by 'ros2 run'：

```
export COLCON_CURRENT_PREFIX=$YOUR_TROS_PATH
source $COLCON_CURRENT_PREFIX/setup.bash
ros2 run imu_sensor imu_sensor --ros-args -p config_file_path:=./install/lib/imu_sensor/config/bmi088.yaml
```
Launched by 'ros2 launch'：
```
export COLCON_CURRENT_PREFIX=$YOUR_TROS_PATH
source $COLCON_CURRENT_PREFIX/setup.bash
ros2 launch imu_sensor imu_sensor.launch.py
```
The config_file_path in which the name, i2c_bus, data range and bandwidth
of imu are configured is read by imu_sensor node. 
The details of the configuration file are mentioned as followed.
```YAML
name: "bmi088"
# i2c_bus serial number
i2c_bus: 1
# accelerometer range, unit 'g'
acc_range: 12
# gyroscope range, unit 'deg/s'
gyro_range: 1000
# accelerometer low pass filter bandwidth
acc_bandwidth: 40
# gyroscope low pass filter bandwidth
gyro_bandwidth: 40
# group_delay of imu,
# which means the latency of the motion of body to data ready, unit 'ms'
group_delay: 7
# imu_data_path from which we read imu data
imu_data_path: "/dev/input/event2"
# imu_virtual_path from which we init imu
imu_virtual_path: "/sys/devices/virtual/input/input0/"
```

---

## X3 linaro
Copy the install directory cross compiled by docker to X3 directory, 
such as /userdata. Then run comand:
````
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`
````

Change ROS_LOG_DIR and run mount -o remount,rw /
```
export ROS_LOG_DIR=/userdata/
mount -o remount,rw /
```

launch imu_sensor
```
#/userdata/install/lib/imu_sensor/imu_sensor --ros-args -p config_file_path:=/userdata/install/lib/config/bmi088.yaml
```

Launched by 'ros2 launch'：
`ros2 launch install/share/mipi_cam/launch/mipi_cam.launch.py`
