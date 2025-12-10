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


#include "imu_sensor/imu_struct.h"
#include <dlfcn.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "imu_manager.hpp"
#include "imu_node.hpp"

namespace imu_sensor
{

imu_node::imu_node(): Node("imu_sensor"), is_running_(true) {

  sensor_type_ = "";

  this->declare_parameter<std::string>("sensor_type", sensor_type_);

  this->get_parameter<std::string>("sensor_type", sensor_type_);


  imu_manager_ = std::make_shared<ImuManager>();
  if (0 != imu_manager_->init_sensor(sensor_type_)) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("imu_sensor"),
              "[%s]->imu node init failure.\n",
              __func__);
    rclcpp::shutdown();
  }

  pub_imu_ = this->create_publisher<
          sensor_msgs::msg::Imu>("/imu_data", 10);

  timer_.emplace_back(std::make_shared<std::thread>([this]() { while(rclcpp::ok()) {this->read_data();}}));

}

imu_node::~imu_node() {
  is_running_ = false;
  for (auto timer : timer_) {
    timer->join();
  }
  timer_.clear();
}


void imu_node::read_data() {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "imu_link";
  ImuData_T imu_data;

  while (is_running_ ) {
    size_t subscriber_count = pub_imu_->get_subscription_count();
    if (subscriber_count > 0) {
      imu_manager_->read_sensor_data(&imu_data);
      imu_msg.header.stamp.set__sec(imu_data.timestamp / 1e9);
      imu_msg.header.stamp.set__nanosec(imu_data.timestamp - imu_msg.header.stamp.sec * 1e9);
      imu_msg.linear_acceleration.x = imu_data.ax;
      imu_msg.linear_acceleration.y = imu_data.ay;
      imu_msg.linear_acceleration.z = imu_data.az;
      imu_msg.angular_velocity.x = imu_data.gx;
      imu_msg.angular_velocity.y = imu_data.gy;
      imu_msg.angular_velocity.z = imu_data.gz;
      pub_imu_->publish(imu_msg);
    }
    usleep(10*1000);
  }
}

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imu_sensor::imu_node>();
  rclcpp::spin(node);
  return 0;
}
