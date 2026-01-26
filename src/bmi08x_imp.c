//
// Created by zhy on 1/22/26.
//
#include "bmi08x.h"
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <stdint.h>
#include <time.h>
#include <linux/input.h>
#include <stdlib.h>

#define LOG_ERR(fmt, ...) \
    do { \
        fprintf(stderr, "[ERROR] [%s: %dh] " fmt "\n", \
                 __FILE__, __LINE__, ##__VA_ARGS__); \
    } while (0)

#define LOG_INFO(fmt, ...) \
    do { \
        fprintf(stdout, "[INFO] [%s: %dh] " fmt "\n", \
                 __FILE__, __LINE__, ##__VA_ARGS__); \
    } while (0)

#define GET_SET_VALUE(fd, rregister, data, set_data, default_data, maxcount, maxcount_num) \
  maxcount = maxcount_num; \
  do { \
       read_iic_register(fd, rregister, default_data); \
       if (set_data != *default_data) { \
         write_iic_register(fd, rregister, set_data);   \
         read_iic_register(fd, rregister, data); \
       } else { *data = *default_data; }  \
       printf(#rregister": [addr: 0x%02x, set value: 0x%02x, read value: 0x%02x, default value: 0x%02x]\n", \
         rregister, set_data, *data, *default_data);\
       if (set_data != *data) { printf("[ERROR] " #rregister" register set failed! \n"); } \
       else { printf("[OK] " #rregister" register set succeed! \n"); break; }    \
  } while(maxcount-- >= 0);


const uint8_t GYRO_INT4_INT3_IO_MAP_REGISTER = 0x18;
const uint8_t GYRO_INT4_INT3_IO_CONF_REGISTER = 0x16;
const uint8_t GYRO_INT_CTRL_REGISTER = 0x15;
const uint8_t GYRO_BANDWIDTH_REGISTER = 0x10;
const uint8_t GYRO_RANGE_REGISTER = 0x0F;

const uint8_t ACC_INT_MAP_DATA_REGISTER = 0x58;
const uint8_t ACC_INT2_IO_CTRL_REGISTER = 0x54;
const uint8_t ACC_INT1_IO_CTRL_REGISTER = 0x53;
const uint8_t ACC_RANGE_REGISTER = 0x41;
const uint8_t ACC_CONF_REGISTER = 0x40;

static int get_iic_device_fd(const char* i2cBus, uint8_t addr) {
  int ret, fd;
  fd = open(i2cBus, O_RDWR);
  if (fd < 0) {
    LOG_ERR("Failed to open I2C bus: %s, address: 0x%02x, ret: %d\n", i2cBus, addr, fd);
    return fd;
  }

  if ((ret = ioctl(fd, I2C_SLAVE_FORCE, addr)) < 0) {
    close(fd);
    LOG_ERR("Failed to set I2C slave address: 0x%02x, ret: %d\n", addr, ret);
    return ret;
  }
  return fd;
}

static int write_iic_register(int fd, uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  return write(fd, buffer, 2);
}

static int read_iic_register(int fd, uint8_t reg, uint8_t* value) {
  int ret = write(fd, &reg, 1);
  if (ret != 1) {
    return ret;
  }
  return read(fd, value, 1);
}

static int close_iic_fd(int fd) {
  if (fd >= 0) close(fd);
  return 0;
}

static int get_imu_iio_event_fd(const char* iio_dev_path) {
  int ret, event_fd;
  int iio_fd = open(iio_dev_path, O_RDONLY);
  if (iio_fd < 0) {
    LOG_ERR("Open IIO device failed: %s, ret: %d\n", iio_dev_path, iio_fd);
    return -1;
  }
  ret = ioctl(iio_fd, IIO_GET_EVENT_FD_IOCTL, &event_fd);
  if (ret < 0 || event_fd < 0) {
    LOG_ERR("Get event fd failed by IIO_GET_EVENT_FD_IOCTL, ret: %d\n", ret);
    close(iio_fd);
    return ret;
  }
  close(iio_fd);
  return event_fd;
}

static int read_iio_sensor_data(
    Bmi08xDevice *device,
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz,
    uint64_t *ts) {
  int ret = 0;
  float cpu_time_used;
  clock_t start, end;
  char buffer[256] = {0};
  iio_event_data event;
  ret = read(device->event_fd, &event, sizeof(event));
  if (ret < 0) {
    LOG_ERR("read failed, fd: %d, ret: %d, errno: %s", device->event_fd, ret, strerror(errno));
    return -1;
  }

  if (ret != sizeof(event)) {
    printf("Event data broken：%d/%zu bytes\n", ret, sizeof(event));
    return -1;
  }

  start = clock();
  device->data_fp = fopen(device->data_node, "r");
  if (device->data_fp != NULL) {
    LOG_INFO("Get IMU_DATA_NODE fp successed");
  } else {
    LOG_ERR("Get IMU_DATA_NODE fp failed, address: %s", device->data_node);
    return -1;
  }
  end = clock();
  cpu_time_used = ((float)(end - start)) / CLOCKS_PER_SEC;
  //  LOG_INFO("fopen consume: %fms", cpu_time_used * 1000);
  if (fgets(buffer, sizeof(buffer) - 1, (FILE*)device->data_fp)) {
    if (strstr(buffer, "invalid") == NULL) {
      ret = sscanf(buffer, "%hd,%hd,%hd,%hd,%hd,%hd,%lu",
                   ax, ay, az, gx, gy, gz, ts);
    }
  } else {
    LOG_ERR("fgets failed");
  }
  start = clock();
  fclose((FILE*)device->data_fp);
  end = clock();
  cpu_time_used = ((float)(end - start)) / CLOCKS_PER_SEC;
  // LOG_INFO("fclose consume: %fms", cpu_time_used * 1000);
  if (ret == 7) {
    return 0;
  } else {
    LOG_ERR("sscanf ret: %d", ret);
    return -1;
  }
}

static int get_imu_input_event_fd(const char* input_dev_path) {
  int ret, event_fd;
  event_fd = open(input_dev_path, O_RDONLY);
  if (event_fd < 0) {
    LOG_ERR("Open input device failed: %s, ret: %d\n", input_dev_path, event_fd);
    return -1;
  }
  return event_fd;
}

#define BMI088_MSC_TS_LOW  0x04
#define BMI088_MSC_TS_HIGH 0x05

static int read_event_sensor_data(
    Bmi08xDevice *device,
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz,
    uint64_t *ts) {
  int event_cout = 0;
  uint32_t ts_low = 0, ts_high = 0;
  struct input_event ev;
  int fd = device->event_fd;
  while (1) {
    int ret = read(fd, &ev, sizeof(ev));
    if (ret != sizeof(ev)) {
      LOG_ERR("read fd: %d failed, ret: %d", fd, ret);
      return -1;
    }
    ++event_cout;
    switch (ev.type) {
      case EV_ABS:
        switch (ev.code) {
          case ABS_X: *ax = ev.value; break;
          case ABS_Y: *ay = ev.value; break;
          case ABS_Z: *az = ev.value; break;
          case ABS_RX: *gx = ev.value; break;
          case ABS_RY: *gy = ev.value; break;
          case ABS_RZ: *gz = ev.value; break;
        }
        break;
      case EV_MSC:
        switch (ev.code) {
          case BMI088_MSC_TS_LOW:
            ts_low = ev.value;
            break;
          case BMI088_MSC_TS_HIGH:
            ts_high = ev.value;
            break;
          default:
            // LOG_ERROR("Un known MSC code: 0x%x, value: 0x%x", ev.code, ev.value);
            break;
        }
        break;
      case EV_SYN:
        if (ev.code == SYN_REPORT) {
          *ts = ((uint64_t)ts_high << 32) | ts_low;
          //LOG_INFO("ts: %u", ((uint64_t)ts_high << 32) | ts_low);
          if (event_cout != 9) LOG_ERR("event_cout: %d", event_cout);
          return 0;
        }
    }
  }
  return 0;
}

static int read_event_sensor_data_v2(
    Bmi08xDevice *device,
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz,
    uint64_t *ts) {
  int event_cout = 0, index = 0, count = 0;
  uint32_t ts_low = 0, ts_high = 0;
  struct input_event ev;
  int fd = device->event_fd;
  while (1) {
    ssize_t ret = read(fd, &ev, sizeof(ev));
    if (ret != sizeof(ev)) {
      LOG_ERR("read failed, ret: %d, sizeof(ev): %d", ret, sizeof(ev));
      return -1;
    }
    if (ev.type != EV_SYN) {
      switch (index++) {
        case 0:
          *ax = ev.value;
          break;
        case 1:
          *ay = ev.value;
          break;
        case 2:
          *az = ev.value;
          break;
        case 3:
          *gx = ev.value;
          break;
        case 4:
          *gy = ev.value;
          break;
        case 5:
          *gz = ev.value;
          break;
        case 6:
          ts_high = ev.value;
          break;
        case 7:
          ts_low = ev.value;
          break;
        case 8:
          count = ev.value;
          break;
        default:
          break;
      }
      if (index == 9) {
        *ts = ((uint64_t)ts_high << 32) | ts_low;
        return 0;
      }
    } else {
      if (index != 0) {
        LOG_ERR("imu recv error, current index: %d", index);
        index = 0;
      }
    }
  }
  return 0;
}

int bmi08x_device_open(Bmi08xDevice *device) {
  int max_retry_count = 5, retry_count;
  char iic_bus_buffer[128];
  snprintf(iic_bus_buffer, sizeof(iic_bus_buffer), "/dev/i2c-%d", device->imu_iic_bus);
  int iic_gyro = get_iic_device_fd(iic_bus_buffer, 0x69);
  int iic_acc = get_iic_device_fd(iic_bus_buffer, 0x19);
  if (iic_gyro <= 0 || iic_acc <= 0) {
    return -1;
  }

  snprintf(iic_bus_buffer, sizeof(iic_bus_buffer),
      "echo 1 > %s/sensor_init;"
      "echo 1 > %s/data_sync;",
      device->virtual_node, device->virtual_node);

  LOG_INFO("excute: \n%s", iic_bus_buffer);
  system(iic_bus_buffer);

  usleep(200 * 1000);

  uint8_t data, set_data, default_data;
  set_data = 0x80;  //  enable int4 pin, disable int3 pin
  GET_SET_VALUE(iic_gyro, GYRO_INT4_INT3_IO_MAP_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);
  usleep(1000 * 100);
  set_data = 0x05;  //  int4 push-pull, active high, int3 push-pull, active high
  GET_SET_VALUE(iic_gyro, GYRO_INT4_INT3_IO_CONF_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);
  usleep(1000 * 100);
  set_data = 0x80;  //  enable new data triggered
  GET_SET_VALUE(iic_gyro, GYRO_INT_CTRL_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);
  usleep(1000 * 100);
  set_data = 0x83; //  400Hz
  GET_SET_VALUE(iic_gyro, GYRO_BANDWIDTH_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);
  usleep(1000 * 100);
  // 0 -> +-2000, 1-> +- 1000, 2-> +- 500, 3-> +- 250, 4-> +- 125
  switch (device->gyro_range) {
    case 1000:
      set_data = 0x01;
      break;
    case 500:
      set_data = 0x02;
      break;
    case 250:
      set_data = 0x03;
      break;
    default:
      set_data = 0x00;
      break;
  }
  GET_SET_VALUE(iic_gyro, GYRO_RANGE_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);
  usleep(1000 * 100);

  //  0x44(01000100) map int2 int1 data ready
  //  0x04(00000100) map int1 data ready
  set_data = 0x04;
  GET_SET_VALUE(iic_acc, ACC_INT_MAP_DATA_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);
  usleep(1000 * 100);
  set_data = 0x16;  //  int2 input pin, activte high, push-pull
  GET_SET_VALUE(iic_acc, ACC_INT2_IO_CTRL_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);
  usleep(1000 * 100);
  set_data = 0x0A;  //  int1 output pin, activte high, push-pull
  GET_SET_VALUE(iic_acc, ACC_INT1_IO_CTRL_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);
  usleep(1000 * 100);
  set_data = 0x02; //  +-12G
  switch (device->acc_range) {
    case 3:
      set_data = 0x00;
      break;
    case 6:
      set_data = 0x01;
      break;
    case 12:
      set_data = 0x02;
      break;
    default:
      set_data = 0x03;
      break;
  }
  GET_SET_VALUE(iic_acc, ACC_RANGE_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);
  usleep(1000 * 100);
  set_data = 0x8A; // OSR4, 400Hz
  GET_SET_VALUE(iic_acc, ACC_CONF_REGISTER, &data, set_data, &default_data, retry_count, max_retry_count);

  close_iic_fd(iic_acc);
  close_iic_fd(iic_gyro);

  if (max_retry_count < 0) {
    LOG_ERR("set register failed!");
    return -1;
  }

  if (strncmp(device->data_node, "/sys/bus/iio", 12) == 0) {
    device->imu_device_type = IMU_DEVICE_TYPE_IIO;
  }

  if (strncmp(device->data_node, "/dev/input", 10) == 0) {
    device->imu_device_type = IMU_DEVICE_TYPE_INPUT;
  }

  if (device->imu_device_type == IMU_DEVICE_TYPE_IIO) {
    device->event_fd = get_imu_iio_event_fd(device->iio_device);
    LOG_INFO("Open iio device: %s succeed, event_fd: %d", device->iio_device, device->event_fd);
  } else if (device->imu_device_type == IMU_DEVICE_TYPE_INPUT) {
    device->event_fd = get_imu_input_event_fd(device->data_node);
    LOG_INFO("Open event device: %s succeed, event_fd: %d", device->data_node, device->event_fd);
  }

  if (device->event_fd < 0) {
    LOG_ERR("Get event fd failed, data_node: %s, device: %s",
            device->data_node, device->iio_device);
    return -1;
  }

  device->gscale = device->gyro_range / (pow(2, 16) * 2.0f - 1) * M_PI / 180.0;
  device->ascale = device->acc_range / pow(2, 16) * 2.0f;

  LOG_INFO("bmi08x_device_open succeed!");
  return 0;
}

int bmi08x_get_frame(Bmi08xDevice *device, Bmi08xFrame *frame) {
  int ret;
  fd_set readfds;
  struct timeval timeout;
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  uint64_t data_ts = 0;
  int event_fd;

  if (device == NULL || frame == NULL) {
    LOG_ERR("Bmi08xDevice or Bmi08xFrame is null!\n");
    return -1;
  }

  FD_ZERO(&readfds);
  FD_SET(device->event_fd, &readfds);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  event_fd = device->event_fd;
  ret = select(event_fd + 1, &readfds, NULL, NULL, &timeout);
  if (ret < 0) {
    LOG_ERR("select failed, ret: %d, errno: %s", ret, strerror(errno));
    return -1;
  }

  if (ret == 0) {
    return -1;
  }

  if (FD_ISSET(event_fd, &readfds)) {
    if (device->imu_device_type == IMU_DEVICE_TYPE_IIO) {
      if (read_iio_sensor_data(device, &ax, &ay, &az, &gx, &gy, &gz, &data_ts) != 0) {
        LOG_ERR("read_sensor_data failed");
        return -1;
      }
    } else if (device->imu_device_type == IMU_DEVICE_TYPE_INPUT) {
      if (read_event_sensor_data_v2(device, &ax, &ay, &az, &gx, &gy, &gz, &data_ts) != 0) {
        LOG_ERR("read_sensor_data failed");
        return -1;
      }
    }
    frame->sys_timestamp = data_ts;
    frame->ax = device->ascale * ax;
    frame->ay = device->ascale * ay;
    frame->az = device->ascale * az;
    frame->gx = device->gscale * gx;
    frame->gy = device->gscale * gy;
    frame->gz = device->gscale * gz;
    //  LOG_INFO("DataTS: %lu | ACC(%d, %d, %d) | GYRO(%d, %d, %d)\n", data_ts, ax, ay, az,gx, gy, gz);
  }
  return 0;
}

int bmi08x_device_close(Bmi08xDevice *device) {
  if (device == NULL) {
    LOG_ERR("Bmi08xDevice or Bmi08xFrame is null!\n");
    return -1;
  }
  close(device->event_fd);
  LOG_INFO("Bmi08xDevice closed, event fd: %d", device->event_fd);
  return 0;
}