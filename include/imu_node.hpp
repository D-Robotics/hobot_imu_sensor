// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HOBOT_IMU_NODE_HPP_
#define HOBOT_IMU_NODE_HPP_

// #include <vector>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "imu_manager.hpp"
#include <sensor_msgs/msg/imu.hpp>


namespace imu_sensor
{

class imu_node : public rclcpp::Node {
 public:
  imu_node();
  ~imu_node();
  void read_data();

private:
    std::atomic_bool is_running_;
    std::string sensor_type_;
    std::string config_file_;
    std::shared_ptr<std::thread> read_thread_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    std::shared_ptr<ImuManager> imu_manager_;
    std::vector<std::shared_ptr<std::thread>> timer_;
};
}  // namespace imu_sensor
#endif  // HOBOT_IMU_NODE_HPP_
