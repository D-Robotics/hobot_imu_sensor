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

class imu_node : public rclcpp::Node {
public:
  imu_node() : Node("imu_sensor"), is_running_(true) {

    std::string tros_distro
        = std::string(std::getenv("TROS_DISTRO")? std::getenv("TROS_DISTRO") : "");
    this->declare_parameter("config_file_path",
            "/opt/tros/" + tros_distro + "/lib/imu_sensor/config/bmi088.yaml");
    this->get_parameter("config_file_path", config_file_);

    std::ifstream fin(config_file_);
    if (!fin.good()) {
      RCLCPP_FATAL(this->get_logger(),"can not find config file: %s",
              config_file_.c_str());
      exit(-1);
    }

    YAML::Node imu_yaml = YAML::Load(fin);
    if (imu_yaml["name"]) {
      imu_name_ = imu_yaml["name"].as<std::string>();
    } else {
      RCLCPP_FATAL(this->get_logger(),
                   "can not find imu setting key: name.");
      exit(-1);
    }

    std::string imu_lib_path = "/opt/tros/" + tros_distro + "/lib/imu_sensor/lib" + imu_name_ + ".so";

    so_handle_ = dlopen(imu_lib_path.c_str(), RTLD_LAZY);
    if (so_handle_ == nullptr) {
      RCLCPP_FATAL(this->get_logger(),
              "dlopen imu sensor path: %s error: %s.",
                   imu_lib_path.c_str(), dlerror());
      exit(-1);
    }
    imu_instance_ = reinterpret_cast<imu_struct*>(
            dlsym(so_handle_, imu_name_.c_str()));
    if (imu_instance_ == nullptr) {
      RCLCPP_FATAL(this->get_logger(),
              "dlsym imu sensor: %s error: %s.",
                   imu_name_.c_str(), dlerror());
      exit(-1);
    }

    pub_imu_ = this->create_publisher<
            sensor_msgs::msg::Imu>("/imu_data", 10);

    read_thread_ = std::make_shared<std::thread>([this]() {
      int ret = imu_instance_->init(config_file_);
      if (ret != 0) {
        RCLCPP_FATAL(this->get_logger(),
                     "imu sensor: %s init error.", imu_name_.c_str());
        dlclose(so_handle_);
        exit(-1);
      }
      read();
      imu_instance_->deinit();
    });
  }

  ~imu_node() {
    shut_down();
  }

  void shut_down() {
    if (!is_running_) return;
    is_running_ = false;
    if (read_thread_) {
      read_thread_->join();
      read_thread_ = nullptr;
    }
    dlclose(so_handle_);
  }

private:
  void read() {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_struct::ImuDataType imu_data;
    while (is_running_) {
      if (imu_instance_->read(imu_data) != 0) {
        continue;
      }
      const auto &msg = imu_data.first;
      const auto &ts = imu_data.second;
      imu_msg.header.stamp.set__sec(ts / 1e9);
      imu_msg.header.stamp.set__nanosec(
              ts - imu_msg.header.stamp.sec * 1e9);
      imu_msg.linear_acceleration.x = msg[0];
      imu_msg.linear_acceleration.y = msg[1];
      imu_msg.linear_acceleration.z = msg[2];
      imu_msg.angular_velocity.x = msg[3];
      imu_msg.angular_velocity.y = msg[4];
      imu_msg.angular_velocity.z = msg[5];
      pub_imu_->publish(imu_msg);
    }
  }

private:
  std::atomic_bool is_running_;
  std::string imu_name_;
  std::string config_file_;
  std::shared_ptr<std::thread> read_thread_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

private:
  void *so_handle_;
  imu_struct *imu_instance_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imu_node>();
  rclcpp::spin(node);
  node->shut_down();
  return 0;
}