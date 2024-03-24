[English](./README.md) | 简体中文

 Getting Started with IMU Sensor Node
---
#Intro
imu_sensor包用于发布sensor_msgs::msg::imu ROS2话题，话题内包括物体运动的角速度和线性加速度以及精调的时间戳。
本文详细介绍了如何编译并使用imu_sensor包。
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
imu_sensor包编译完毕后，config和launch目录以及.so库分别安装在
install/lib/imu_sensor or install/share/imu_sensor 目录下。

## Compiling
包支持X3板端编译和PC端的交叉编译。
### Compiling on X3 in Ubuntu
1、确认编译环境
- X3安装Ubuntu系统
- source TogetheROS bash file `source $TogetheROS_PATH/setup.bash` 其中$TogetheROS_PATH 是TogetheROS的安装目录
- colcon 已安装完毕, 否则 :`pip install -U colcon-common-extensions`

2、编译：
  `colcon build --packages-select imu_sensor`。


### Cross compiling on PC in docker 

1、确认编译环境
- 参考这篇文档安装编译环境
 (https://c-gitlab.horizon.ai/HHP/robot_dev_config/-/blob/develop/README.md)


2、编译

- 编译命令： 

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

其中 config_file_path 是配置文件, 配置文件中的字段i2c_bus, data range 以及 bandwidth的含义如下所示。
```YAML
name: "bmi088"
# i2c_bus 总线号
i2c_bus: 1
# 加速度计量程, 单位 'g'
acc_range: 12
# 陀螺仪量程, 单位 'deg/s'
gyro_range: 1000
# 加速度计低通滤波器带宽
acc_bandwidth: 40
# 陀螺仪低通滤波器带宽
gyro_bandwidth: 40
# group_delay of imu,
# which means the latency of the motion of body to data ready, 单位 'ms'
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
