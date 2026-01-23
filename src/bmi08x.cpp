
//#include "bmi08x.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <sys/select.h>
#include <stdint.h>
#include <time.h>

struct iio_event_data {
  unsigned long long id;
  long long timestamp;
};

#define IIO_GET_EVENT_FD_IOCTL _IOR('i', 0x90, int)

#define IMU_IIO_DEV_PATH     "/dev/iio:device1"
#define IMU_DATA_NODE        "/sys/bus/iio/devices/iio:device1/bmi088_latest_data"

#define GET_SET_VALUE(fd, rregister, data, set_data, default_data) \
  read_iic_register(fd, rregister, default_data); \
  write_iic_register(fd, rregister, set_data);   \
  read_iic_register(fd, rregister, data); \
  printf(#rregister": [addr: 0x%02x, set value: 0x%02x, read value: 0x%02x, default value: 0x%02x]\n", \
         rregister, set_data, *data, *default_data);\
  if (set_data != *data) { printf(#rregister"[ERROR] register set failed! \n"); }

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


int get_iic_device_fd(const char* i2cBus, uint8_t addr) {
  int ret, fd;
  fd = open(i2cBus, O_RDWR);
  if (fd < 0) {
    printf("Failed to open I2C bus: %s, address: 0x%02x, ret: %d\n", i2cBus, addr, fd);
    return fd;
  }

  if ((ret = ioctl(fd, I2C_SLAVE_FORCE, addr)) < 0) {
    close(fd);
    printf("Failed to set I2C slave address: 0x%02x, ret: %d\n", addr, ret);
    return ret;
  }
  return fd;
}

int write_iic_register(int fd, uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  return write(fd, buffer, 2);
}

int read_iic_register(int fd, uint8_t reg, uint8_t* value) {
  int ret = write(fd, &reg, 1);
  if (ret != 1) {
    return ret;
  }
  return read(fd, value, 1);
}

int close_iic_fd(int fd) {
  if (fd >= 0) close(fd);
  return 0;
}

int get_imu_event_fd(const char* iio_dev_path) {
  int ret, event_fd;
  // 1. 打开IIO设备，获取Event FD
  int iio_fd = open(iio_dev_path, O_RDONLY);
  if (iio_fd < 0) {
    printf("打开IIO设备失败: %s, ret: %d\n", iio_dev_path, iio_fd);
    return 1;
  }

  ret = ioctl(iio_fd, IIO_GET_EVENT_FD_IOCTL, &event_fd);
  if (ret < 0 || event_fd < 0) {
    printf("获取Event FD失败 by IIO_GET_EVENT_FD_IOCTL, ret: %d\n", ret);
    close(iio_fd);
    return ret;
  }
  close(iio_fd); // 拿到Event FD后，原设备FD可关闭
  return event_fd;
}

static int read_sensor_data(
    FILE *fp,
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz,
    uint64_t *ts) {
  if (!fp) {
    perror("读取数据节点失败");
    return -1;
  }
  char buffer[256] = {0};
  int ret = 0;
  if (fgets(buffer, sizeof(buffer) - 1, fp)) {
    if (strstr(buffer, "invalid") == NULL) {
      ret = sscanf(buffer, "%hd,%hd,%hd,%hd,%hd,%hd,%lu",
                   ax, ay, az, gx, gy, gz, ts);
    }
  } else {
    printf("fgets failed\n");
  }
  if (ret == 7) {
    return 0;
  } else {
    printf("sscanf ret: %d\n", ret);
    return -1;
  }
}

static volatile int keep_running = 1;

void int_handler(int sig) {
  keep_running = 0;
  signal(sig, int_handler);
}

int main(int argc, char **argv) {
  FILE *imu_fp;
  fd_set readfds;
  int ret, imu_event_fd;
  int16_t ax, ay, az, gx, gy, gz;
  uint64_t data_ts, last_data_ts = 0;
  int64_t diff, min_diff = INT64_MAX, max_diff = INT64_MIN;
  uint32_t event_count = 0, event_lost_count = 0, ts_lost_count = 0;
  struct sigaction sa;
  struct iio_event_data event, last_event;
  struct timeval timeout;
  clock_t start, end;
  float cpu_time_used;

  int iic_gyro = get_iic_device_fd("/dev/i2c-5", 0x69);
  int iic_acc = get_iic_device_fd("/dev/i2c-5", 0x19);

  uint8_t data, set_data, default_data;
  set_data = 0x80;  //  enable int4 pin, disable int3 pin
  GET_SET_VALUE(iic_gyro, GYRO_INT4_INT3_IO_MAP_REGISTER, &data, set_data, &default_data);
  usleep(1000 * 100);
  set_data = 0x0A;  //  int4 push-pull, active high, int3 push-pull, active high
  GET_SET_VALUE(iic_gyro, GYRO_INT4_INT3_IO_CONF_REGISTER, &data, set_data, &default_data);
  usleep(1000 * 100);
  set_data = 0x80;  //  enable new data triggered
  GET_SET_VALUE(iic_gyro, GYRO_INT_CTRL_REGISTER, &data, set_data, &default_data);
  usleep(1000 * 100);
  set_data = 0x83; //  400Hz
  GET_SET_VALUE(iic_gyro, GYRO_BANDWIDTH_REGISTER, &data, set_data, &default_data);
  usleep(1000 * 100);
  set_data = 0x01; // 0 -> +-2000, 1-> +- 1000, 2-> +- 500, 3-> +- 250, 4-> +- 125
  GET_SET_VALUE(iic_gyro, GYRO_RANGE_REGISTER, &data, set_data, &default_data);
  usleep(1000 * 100);

  //  0x44(01000100) map int2 int1 data ready
  //  0x04(00000100) map int1 data ready
  set_data = 0x44;
  GET_SET_VALUE(iic_acc, ACC_INT_MAP_DATA_REGISTER, &data, set_data, &default_data);
  usleep(1000 * 100);
  set_data = 0x09;  //  int2 input pin, activte high, push-pull
  GET_SET_VALUE(iic_acc, ACC_INT2_IO_CTRL_REGISTER, &data, set_data, &default_data);
  usleep(1000 * 100);
  set_data = 0x0A;  //  int1 output pin, activte high, push-pull
  GET_SET_VALUE(iic_acc, ACC_INT1_IO_CTRL_REGISTER, &data, set_data, &default_data);
  usleep(1000 * 100);
  set_data = 0x02; //  +-12G
  GET_SET_VALUE(iic_acc, ACC_RANGE_REGISTER, &data, set_data, &default_data);
  usleep(1000 * 100);
  set_data = 0x8A; // OSR4, 400Hza
  GET_SET_VALUE(iic_acc, ACC_CONF_REGISTER, &data, set_data, &default_data);

  close_iic_fd(iic_acc);
  close_iic_fd(iic_gyro);

  sa.sa_handler = int_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  // 打印程序信息
  printf("========================================\n");
  printf("BMI088 纯Event监听工具\n");
  printf("设备路径：%s\n", IMU_IIO_DEV_PATH);
  printf("数据节点：%s\n", IMU_DATA_NODE);
  printf("驱动数据更新时自动打印 | 按Ctrl+C退出\n");
  printf("========================================\n\n");

  imu_event_fd = get_imu_event_fd(IMU_IIO_DEV_PATH);
  if (imu_event_fd > 0) {
    printf("成功获取Event FD: %d\n", imu_event_fd);
    printf("开始监听驱动Event...\n\n");
  } else {
    printf("[ERROR] Failed 获取EventFD, address: %s\n", IMU_IIO_DEV_PATH);
    return -1;
  }

  while (keep_running) {
    FD_ZERO(&readfds);
    FD_SET(imu_event_fd, &readfds);
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    start = clock();
    imu_fp = fopen(IMU_DATA_NODE, "r");
    if (imu_fp != NULL) {
      printf("Successed getting IMU_DATA_NODE FP\n");
    } else {
      printf("[ERROR] Failed getting IMU_DATA_NODE FP, address: %s\n", IMU_DATA_NODE);
      return -1;
    }
    end = clock();
    cpu_time_used = ((float)(end - start)) / CLOCKS_PER_SEC;
    printf("fopen consume: %fms\n", cpu_time_used * 1000);
    // 阻塞等待Event（有事件才返回）
    ret = select(imu_event_fd + 1, &readfds, NULL, NULL, &timeout);
    if (ret < 0) {
      if (errno == EINTR && !keep_running) break;
      perror("select监听失败");
      continue;
    }

    // 超时：驱动无数据更新
    if (ret == 0) {
      continue;
    }

    if (FD_ISSET(imu_event_fd, &readfds)) {
      ret = read(imu_event_fd, &event, sizeof(event));
      if (ret < 0) {
        if (errno == EINTR && !keep_running) break;
        perror("读取Event失败");
        continue;
      }

      if (ret != sizeof(event)) {
        printf("Event数据不完整：%d/%zu字节\n", ret, sizeof(event));
        continue;
      }
      event_count++;
      if (event.id - last_event.id > 1) {
        event_lost_count += (event.id - last_event.id);
        printf("[ERROR] Detect event data lost! last event id: %llu, current id: %llu\n",
               last_event.id, event.id);
      }
      if (read_sensor_data(imu_fp, &ax, &ay, &az, &gx, &gy, &gz, &data_ts) == 0) {
        diff = data_ts - last_data_ts;
        if (last_data_ts != 0 && diff > 0.003) {
          ts_lost_count++;
          printf("[ERROR] Detect imu data lost!, last ts: %fs, current ts: %fs, diff: %fs\n",
                 last_data_ts * 1e-9, data_ts * 1e-9, diff * 1e-9);
        }
        if (diff < min_diff) min_diff = diff;
        if (diff > max_diff) max_diff = diff;
        printf("Event ID: %llu | Event TS: %lld | DataTS: %lu | ACC(%d, %d, %d) | GYRO(%d, %d, %d) | DIFF(%fs, %fs, %fs) | LOST(%u, %u)\n",
               event.id, event.timestamp, data_ts,
               ax, ay, az, gx, gy, gz, diff * 1e-9, min_diff * 1e-9, max_diff * 1e-9, event_lost_count, ts_lost_count);
      } else {
        printf("Event#%d | 读取数据失败\n", event_count);
      }
      last_event = event;
      last_data_ts = data_ts;
      start = clock();
      fclose(imu_fp);
      end = clock();
      cpu_time_used = ((float)(end - start)) / CLOCKS_PER_SEC;
      printf("fclose consume: %fms\n", cpu_time_used * 1000);
    }
  }

  close(imu_event_fd);
  printf("\n程序退出，共收到 %d 个Event\n", event_count);
  return 0;
}
