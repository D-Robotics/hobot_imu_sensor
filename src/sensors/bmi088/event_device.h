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
#include <fcntl.h>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <linux/input.h>
#include <linux/input-event-codes.h>

#include <atomic>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <deque>

#pragma once

struct bmi088 : public imu_struct {
  int init(const std::string& config_file) override;
  int read(ImuDataType &imu_data) override;
  int deinit() override;

private:
  uint group_delay_ = 7 * 1e6;
  std::string acc_addr = "0x19";
  std::string gyro_addr = "0x69";
  std::string imu_data_path_ = "/dev/input/event2";
  std::string imu_virtual_path_ = "/sys/devices/virtual/input/input0/";
  uint8_t i2c_bus = 1;
  int event_fd_ = -1;
  std::atomic_bool is_running_;
  int gyro_range_ = 1000;
  int acc_range_ = 12;
  int acc_bandwidth_ = 40;
  int gyro_bandwidth_ = 40;

private:
  std::mutex mtx_;
  std::condition_variable cv_;
  std::shared_ptr<std::thread> read_thread_;
  std::deque<ImuDataType> que_;
  void PollTread();
  int write_node(const char *node, char *val, int num);
};
