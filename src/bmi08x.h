//
// Created by zhy on 1/21/26.
//



#ifndef DROBOTICS_VIO_SENSOR_BMI08X_H_
#define DROBOTICS_VIO_SENSOR_BMI08X_H_

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#define IIO_GET_EVENT_FD_IOCTL _IOR('i', 0x90, int)
#define IMU_IIO_DEV_PATH     "/dev/iio:device1"
#define IMU_DATA_NODE        "/sys/bus/iio/devices/iio:device1/bmi088_latest_data"
#define IMU_INPUT_DEV_PATH "/dev/input/event3"

typedef enum {
  IMU_DEVICE_TYPE_INPUT,
  IMU_DEVICE_TYPE_IIO
} IMU_DEVICE_TYPE;

typedef struct {
  unsigned long long id;
  long long timestamp;
} iio_event_data;

typedef struct {
  /* configuration */
  IMU_DEVICE_TYPE imu_device_type;
  const char* iio_device;
  const char* data_node;
  const char* virtual_node;
  int imu_iic_bus;
  int acc_range;
  int acc_bandwidth;
  int gyro_range;
  int gyro_bandwidth;
  bool io_interrupt;
  /* configuration */

  /* info for get imu frame */
  int event_fd;
  void *data_fp;
  double gscale, ascale;
  /* info for get imu frame */
} Bmi08xDevice;

typedef struct {
  double ax, ay, az;
  double gx, gy, gz;
  uint64_t sys_timestamp;
  iio_event_data event;
} Bmi08xFrame;

int bmi08x_device_open(Bmi08xDevice *device);
int bmi08x_get_frame(Bmi08xDevice *device, Bmi08xFrame *frame, bool use_poll);
int bmi08x_device_close(Bmi08xDevice *device);

#ifdef __cplusplus
}
#endif

#endif // DROBOTICS_VIO_SENSOR_BMI08X_H_
