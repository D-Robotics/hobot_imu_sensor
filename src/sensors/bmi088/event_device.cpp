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
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include "event_device.h"

#define GRAVITY_EARTH (9.80665f)

static int imu_mod_install() {
  char buffer[128];
  FILE *fp = popen("lsmod | grep bmi08", "r");
  if (fp == nullptr) {
    printf("failed to run command \"lsmod | grep bmi08\".\n");
    return errno;
  }
  int ret = fread(buffer, sizeof(char), sizeof(buffer), fp);
  if (ret != 0 && strncmp(buffer, "bmi08", 5) == 0) {
    printf("imu driver is already installed.\n");
  } else {
    printf("install imu driver.\n");
    system("modprobe bmi088");
  }
  pclose(fp);
  return 0;
}

static int imu_priorities_set() {
  char buffer[128];
  char cmd[128];
  int imu_irq_pid = 0;
  FILE *fp = popen("ps -eLo pid,cmd "
                   "| grep -i \"irq/161-bmi08\" | grep -v grep", "r");
  int ret = fread(buffer, sizeof(char), sizeof(buffer), fp);
  if (ret != 0 && sscanf(buffer, "%d", &imu_irq_pid) == 1) {
    sprintf(cmd, "chrt -f -p 98 %d", imu_irq_pid);
    system(cmd);
    sprintf(cmd, "chrt -p %d", imu_irq_pid);
    system(cmd);
  } else {
    printf("failed to get irq of imu number\n");
    return -1;
  }
  return 0;
}

static void write_imu_i2c(
        uint8_t i2c_bus,
        const char* addr, const char* register_addr, uint8_t value) {
  char cmd[128];
  printf("-------\n");
  sprintf(cmd, "i2cget -f -y %d %s %s",
          i2c_bus, addr, register_addr);
  system(cmd);
  sprintf(cmd, "i2cset -f -y %d %s %s 0x%x",
          i2c_bus, addr, register_addr,
          value);
  system(cmd);
  printf("set imu: %s, result: \n", cmd);
  sprintf(cmd, "i2cget -f -y %d %s %s",
          i2c_bus, addr, register_addr);
  system(cmd);
  printf("-------\n");
}

static int imu_range_set(
        uint8_t i2c_bus,
        const char*acc_addr, const char *gyro_addr,
        int acc_range, int gyro_range) {
  uint acc_register_value = 0x03;
  uint gyro_register_value = 0x00;
  const char* acc_range_addr = "0x41";
  const char* gyro_range_addr = "0x0F";
  switch (acc_range) {
    case 24 :
      break;
    case 12:
      acc_register_value -= 1;
      break;
    case 6:
      acc_register_value -= 2;
      break;
    case 3:
      acc_register_value -= 3;
      break;
    default:
      printf("unsupport accelerometer range: %d\n", acc_range);
      exit(-1);
  }
  switch (gyro_range) {
    case 2000 :
      break;
    case 1000:
      gyro_register_value += 1;
      break;
    case 500:
      gyro_register_value += 2;
      break;
    case 250:
      gyro_register_value += 3;
      break;
    case 125:
      gyro_register_value += 4;
      break;
    default:
      printf("unsupport gyroscope range: %d\n", gyro_range);
      exit(-1);
  }
  write_imu_i2c(i2c_bus, acc_addr, acc_range_addr, acc_register_value);
  write_imu_i2c(i2c_bus, gyro_addr, gyro_range_addr, gyro_register_value);
  return 0;
}

static int imu_filter_set(uint8_t i2c_bus,
        const char*acc_addr, const char *gyro_addr,
        uint acc_bandwidth, uint gyro_bandwidth) {
  uint8_t acc_bwp = 0x0a;
  uint8_t acc_odr = 0x0a;
  const char *acc_band_addr = "0x40";
  (void)gyro_addr;
  (void)gyro_bandwidth;
  //  const char *gyro_band_addr = "0x10";
  switch (acc_bandwidth) {
    case 145:
      break;
    case 75:
      acc_bwp -= 1;
      break;
    case 40:
      acc_bwp -= 2;
      break;
  }
  write_imu_i2c(i2c_bus, acc_addr, acc_band_addr,
           acc_bwp << 4 | acc_odr);
  return 0;
}


static uint8_t get_uc_range (int range) {
  switch (range) {
    case 2000:
      return UINT8_C(0x00);
    case 1000:
      return UINT8_C(0x01);
    case 500:
      return UINT8_C(0x02);
    case 250:
      return UINT8_C(0x03);
    case 125:
      return UINT8_C(0x04);
  }
  return 0;
}

static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width, uint8_t range) {
  float half_scale = ((float)(1 << bit_width) / 2.0f);
  return (dps / ((half_scale) + range)) * (val);
}

static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width) {
  float gravity;
  float half_scale = ((1 << bit_width) / 2.0f);
  gravity = (float)((GRAVITY_EARTH * val * g_range) / half_scale);
  return gravity;
}

ssize_t						/* Read "n" bytes from a descriptor. */
readn(int fd, void *vptr, size_t n) {
  size_t	nleft;
  ssize_t	nread;
  char	*ptr;

  ptr = static_cast<char*>(vptr);
  nleft = n;
  while (nleft > 0) {
    if ( (nread = read(fd, ptr, nleft)) < 0) {
      if (errno == EINTR)
        nread = 0;		/* and call read() again */
      else
        return(-1);
    } else if (nread == 0)
      break;				/* EOF */

    nleft -= nread;
    ptr   += nread;
  }
  return(n - nleft);		/* return >= 0 */
}

void bmi088::PollTread() {
  int ret;
  int index = 0, count, last_count = 0;
  float imu[6];
  uint32_t time_lsb, time_msb;
  input_event event{};
  uint64_t lost = 0;
  while (is_running_) {
    struct pollfd pfd;
    pfd.fd = event_fd_;
    pfd.events = POLLIN;
    ret = poll(&pfd, 1, 200);
    if (ret <= 0) {
      printf("poll failed: %d!\n", ret);
      continue;
    }
    if ((ret = readn(event_fd_, &event, sizeof(event))) == sizeof(event)) {
      if (event.type != EV_SYN) {
        switch (index++) {
          case 0:
            imu[0] = -lsb_to_mps2(event.value, acc_range_, 16);
            break;
          case 1:
            imu[1] = -lsb_to_mps2(event.value, acc_range_, 16);
            break;
          case 2:
            imu[2] = lsb_to_mps2(event.value, acc_range_, 16);
            break;
          case 3:
            imu[3] = -lsb_to_dps(event.value, gyro_range_,
                    16, get_uc_range(gyro_range_)) * M_PI / 180;
            break;
          case 4:
            imu[4] = -lsb_to_dps(event.value, gyro_range_,
                    16, get_uc_range(gyro_range_)) * M_PI / 180;
            break;
          case 5:
            imu[5] = lsb_to_dps(event.value, gyro_range_,
                    16, get_uc_range(gyro_range_)) * M_PI / 180;
            break;
          case 6:
            time_msb = event.value;
            break;
          case 7:
            time_lsb = event.value;
            break;
          case 8:
            count = event.value;
            break;
          default:
            break;
        }
        if (index == 9) {
          uint64_t sync_time = time_msb;
          sync_time <<= 32;
          sync_time |= time_lsb;
          index = 0;
          if (last_count != 0 && count != last_count + 1) {
            printf("current count: %d, last count: %d, gap: %d\n",
                    count, last_count, count - last_count);
          }
          last_count = count;
          sync_time -= group_delay_;
          std::vector<float> vimu(imu, imu + 6);
          std::pair<std::vector<float>, uint64_t> pimu;
          pimu.first = vimu;
          pimu.second = sync_time;
          std::lock_guard<std::mutex> lockGuard(mtx_);
          que_.emplace_back(std::move(pimu));
          cv_.notify_one();
        }
      } else {
        if (index != 0) {
          ++lost;
          printf("imu recv error, current index: %d\n", index);
          index = 0;
        }
        //printf("l: %llu\n", lost);
      }
    } else {
      printf( "read bytes: %d, but deserve: %ld\n", ret, sizeof(event));
    }
  }
}

int bmi088::write_node(
        const char* node, char *val, int num) {
  int ret = 0;
  char *node_full_path = nullptr;
  ret = asprintf(&node_full_path, "%s%s", imu_virtual_path_.c_str(), node);
  if (ret < 0) {
    return -ENOMEM;
  }
  int fd = open(node_full_path, O_RDWR);
  if (fd < 0) {
    ret = -errno;
    printf( "Failed to open %s, ret: %d\n", node_full_path, fd);
    goto error;
  }
  ret = write(fd, val, num);
  if (ret < 0) {
    printf("Failed to write %s, ret: %d\n", node_full_path, ret);
    goto error;
  }
  error:
  close(fd);
  free(node_full_path);
  return ret;
}

#define GET_YAML_INT(key, value) \
if (imu_yaml[key]) {\
  value = imu_yaml[key].as<int>();\
  printf("%s: %d\n", key, value);\
} else {\
  printf("can not find imu setting key: %s, exit!\n", key);\
  exit(-1);\
}

#define GET_YAML_STRING(key, value) \
if (imu_yaml[key]) {\
  value = imu_yaml[key].as<std::string>();\
  printf("%s: %s\n", key, value.c_str());\
} else {\
  printf("can not find imu setting key: %s, exit!\n", key);\
  exit(-1);\
}

int bmi088::init(const std::string &config_file) {
  int ret;
  char buf = '1';
  std::ifstream fin(config_file);
  if (!fin.good()) {
    printf("can not find config file: %s\n",
            config_file.c_str());
    return -1;
  }
  YAML::Node imu_yaml = YAML::Load(fin);

  printf("bmi088 configuration:\n");
  GET_YAML_INT("i2c_bus", i2c_bus);
  GET_YAML_INT("acc_range", acc_range_);
  GET_YAML_INT("gyro_range", gyro_range_);
  GET_YAML_INT("acc_bandwidth", acc_bandwidth_);
  GET_YAML_INT("gyro_bandwidth", gyro_bandwidth_);
  GET_YAML_INT("group_delay", group_delay_);
  GET_YAML_STRING("imu_data_path", imu_data_path_);
  GET_YAML_STRING("imu_virtual_path", imu_virtual_path_);

  group_delay_ = group_delay_ * 1e6;

  imu_mod_install();
  ret = write_node("sensor_init", &buf, 1);
  if (ret < 0) {
    exit(-1);
  }
  ret = write_node("data_sync", &buf, 1);
  if (ret < 0) {
    exit(-1);
  }
  imu_filter_set(i2c_bus,
                 acc_addr.c_str(), gyro_addr.c_str(), acc_bandwidth_, gyro_bandwidth_);
  imu_range_set(i2c_bus,
                acc_addr.c_str(), gyro_addr.c_str(), acc_range_, gyro_range_);
  imu_priorities_set();
  event_fd_ = open(imu_data_path_.c_str(), O_RDONLY);
  if (event_fd_ < 0) {
    printf( "Fail to open device:%s.\n"
                    "Please confirm the path or you have permission to do this.\n",
            imu_data_path_.c_str());
    exit(-1);
  }
  is_running_ = true;
  read_thread_ = std::make_shared<std::thread>([this] { PollTread(); });
  return 0;
}

int bmi088::read(ImuDataType &imu_data) {
  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck, [&]() {
    return !que_.empty() || !is_running_;
  });
  if (!is_running_) return -1;
  imu_data = que_.front();
  que_.pop_front();
  return 0;
}

int bmi088::deinit() {
  if (!is_running_) return -1;
  is_running_ = false;
  if (read_thread_) {
    cv_.notify_all();
    read_thread_->join();
    read_thread_ = nullptr;
  }
  close(event_fd_);
  return 0;
}

bmi088 bmi088;
