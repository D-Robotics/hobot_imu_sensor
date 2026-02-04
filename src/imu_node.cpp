//
// Created by zhy on 1/22/26.
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fstream>
#include "blockque.h"
#include "bmi08x.h"

#define DECLARE_PARAMETER(name, default_value, p) \
p = this->declare_parameter(name, default_value); \
RCLCPP_WARN_STREAM(this->get_logger(), name << ": " << p);

namespace drobotics
{

/**
 * @class DFMatchComponent
 * @brief A ROS2 node that performs keypoints estimation and matching using DFmatch model.
 */
struct ImuComponent : public rclcpp::Node {
public:
  explicit ImuComponent( const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions(),
                             const std::string &node_name = "ImuComponent");
  ~ImuComponent();

private:
  // ============================================ member functions ============================================
  void set_node_params();
  void set_subscription_publisher();
  void set_imu_instance();
  void set_worker_thread();
  void recv_func();
  void pub_func();

private:
  Bmi08xDevice bmi08x_device_;
private:
  std::string imu_pub_topic_ = "bmi08x_imu", iio_device_ = IMU_IIO_DEV_PATH,
  data_node_ = IMU_INPUT_DEV_PATH, virtual_node_ = "/sys/devices/virtual/input/input1/";
  std::string imu_frame_id_ = "imu_bmi088";
  int iic_bus_ = 5;
  int acc_range = 12, gyro_range = 1000, acc_bandwidth = 40, gyro_bandwidth = 40, group_delay = 7;
  double gravity_ = 9.80665;
  bool imu_adjust_interrupt_ = false, imu_use_pool_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_ = nullptr;
  std::shared_ptr<std::thread> pub_thread_, recv_thread_;
  blockqueue<Bmi08xFrame> frame_que_;
};

ImuComponent::ImuComponent(const rclcpp::NodeOptions &node_options, const std::string &node_name)
    : Node(node_name, node_options) {
  set_node_params();
  set_subscription_publisher();
  set_imu_instance();
  set_worker_thread();
}

ImuComponent::~ImuComponent() {
  if (pub_thread_) {
    pub_thread_->join();
    pub_thread_ = nullptr;
  }
  if (recv_thread_) {
    recv_thread_->join();
    recv_thread_ = nullptr;
  }
  bmi08x_device_close(&bmi08x_device_);
}

void ImuComponent::set_node_params() {
  DECLARE_PARAMETER("imu_pub_topic", imu_pub_topic_, imu_pub_topic_);
  DECLARE_PARAMETER("imu_iio_device", iio_device_, iio_device_);
  DECLARE_PARAMETER("imu_data_node", data_node_, data_node_);
  DECLARE_PARAMETER("imu_virtual_node", virtual_node_, virtual_node_);
  DECLARE_PARAMETER("imu_iic_bus", iic_bus_, iic_bus_);
  DECLARE_PARAMETER("imu_acc_range", acc_range, acc_range);
  DECLARE_PARAMETER("imu_acc_bandwidth", acc_bandwidth, acc_bandwidth);
  DECLARE_PARAMETER("imu_gyro_range", gyro_range, gyro_range);
  DECLARE_PARAMETER("imu_gyro_bandwidth", gyro_bandwidth, gyro_bandwidth);
  DECLARE_PARAMETER("imu_group_delay", group_delay, group_delay);
  DECLARE_PARAMETER("imu_gravity", gravity_, gravity_);
  DECLARE_PARAMETER("imu_frame_id", imu_frame_id_, imu_frame_id_);
  DECLARE_PARAMETER("imu_adjust_interrupt", imu_adjust_interrupt_, imu_adjust_interrupt_);
  DECLARE_PARAMETER("imu_use_pool", imu_use_pool_, imu_use_pool_);
}

void ImuComponent::set_subscription_publisher() {
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_pub_topic_, 400);
}

void ImuComponent::set_imu_instance() {
  int ret;
  bmi08x_device_.iio_device = iio_device_.c_str();
  bmi08x_device_.data_node = data_node_.c_str();
  bmi08x_device_.virtual_node = virtual_node_.c_str();
  bmi08x_device_.imu_iic_bus = iic_bus_;
  bmi08x_device_.acc_range = acc_range;
  bmi08x_device_.acc_bandwidth = acc_bandwidth;
  bmi08x_device_.gyro_range = gyro_range;
  bmi08x_device_.gyro_bandwidth = gyro_bandwidth;
  bmi08x_device_.io_interrupt = imu_adjust_interrupt_;
  ret = bmi08x_device_open(&bmi08x_device_);
  std::cout << std::flush;
  if (ret != 0) {
    RCLCPP_FATAL(get_logger(), "bmi08x_device_open failed");
    rclcpp::shutdown();
  }
}

void ImuComponent::set_worker_thread() {
  pub_thread_ = std::make_shared<std::thread>([this] { pub_func(); });
  recv_thread_ = std::make_shared<std::thread>([this] { recv_func(); });
}

void ImuComponent::pub_func() {
  int ret = 0;
  int64_t diff, min_diff = INT64_MAX, max_diff = INT64_MIN;
  uint64_t lost_count = 0, disorder_count = 0, repeated_count = 0;
  Bmi08xFrame current_frame, last_frame;
  last_frame.sys_timestamp = 0;
  sensor_msgs::msg::Imu imu_msg;
  FILE *file = fopen("imu_error.log", "w");
  if (file == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "Can not create imu_error.log");
  }
  imu_msg.orientation.x = 0;
  imu_msg.orientation.y = 0;
  imu_msg.orientation.z = 0;
  imu_msg.orientation.w = 1;
  imu_msg.header.frame_id = imu_frame_id_;

  while (rclcpp::ok()) {
    if (frame_que_.get(current_frame)) {
      current_frame.sys_timestamp -= group_delay * 1e6;
      imu_msg.header.stamp.set__sec(current_frame.sys_timestamp / 1e9);
      imu_msg.header.stamp.set__nanosec(current_frame.sys_timestamp - imu_msg.header.stamp.sec * 1e9);
      imu_msg.linear_acceleration.x = current_frame.ax * gravity_;
      imu_msg.linear_acceleration.y = current_frame.ay * gravity_;
      imu_msg.linear_acceleration.z = current_frame.az * gravity_;
      imu_msg.angular_velocity.x = current_frame.gx;
      imu_msg.angular_velocity.y = current_frame.gy;
      imu_msg.angular_velocity.z = current_frame.gz;
      imu_pub_->publish(imu_msg);
      diff = current_frame.sys_timestamp - last_frame.sys_timestamp;
      if (last_frame.sys_timestamp != 0 && diff * 1e-9 > 0.003) {
        lost_count++;
        RCLCPP_ERROR(get_logger(), "Lost imu data!, last ts: %fs, current ts: %fs, diff: %fs,"
                                   "lost_count: %lu, disorder_count: %lu, repeated_count: %lu\n",
                     last_frame.sys_timestamp * 1e-9, current_frame.sys_timestamp * 1e-9, diff * 1e-9,
                     lost_count, disorder_count, repeated_count);
        if (file) {
          fprintf(file, "Lost imu data!, last ts: %fs, current ts: %fs, diff: %fs,"
                        "lost_count: %lu, disorder_count: %lu, repeated_count: %lu\n",
                  last_frame.sys_timestamp * 1e-9, current_frame.sys_timestamp * 1e-9, diff * 1e-9,
                  lost_count, disorder_count, repeated_count);
        }
      }
      if (diff < 0) {
        disorder_count++;
        RCLCPP_ERROR(get_logger(), "Disorder imu data!, last ts: %fs, current ts: %fs, diff: %fs,"
                                   "lost_count: %lu, disorder_count: %lu, repeated_count: %lu\n",
                     last_frame.sys_timestamp * 1e-9, current_frame.sys_timestamp * 1e-9, diff * 1e-9,
                     lost_count, disorder_count, repeated_count);
        if (file) {
          fprintf(file, "Disorder imu data!, last ts: %fs, current ts: %fs, diff: %fs,"
                        "lost_count: %lu, disorder_count: %lu, repeated_count: %lu\n",
                  last_frame.sys_timestamp * 1e-9, current_frame.sys_timestamp * 1e-9, diff * 1e-9,
                  lost_count, disorder_count, repeated_count);
        }
      }
      if (diff == 0) {
        repeated_count++;
        RCLCPP_ERROR(get_logger(), "Repeated imu data!, last ts: %fs, current ts: %fs, diff: %fs,"
                                   "lost_count: %lu, disorder_count: %lu, repeated_count: %lu\n",
                     last_frame.sys_timestamp * 1e-9, current_frame.sys_timestamp * 1e-9, diff * 1e-9,
                     lost_count, disorder_count, repeated_count);
        if (file) {
          fprintf(file, "Repeated imu data!, last ts: %fs, current ts: %fs, diff: %fs,"
                        "lost_count: %lu, disorder_count: %lu, repeated_count: %lu\n",
                  last_frame.sys_timestamp * 1e-9, current_frame.sys_timestamp * 1e-9, diff * 1e-9,
                  lost_count, disorder_count, repeated_count);
        }
      }
      if (diff < min_diff) min_diff = diff;
      if (diff > max_diff) max_diff = diff;

      RCLCPP_INFO(get_logger(),
                  "DataTS: %lu | ACC(%f, %f, %f) | GYRO(%f, %f, %f) | DIFF(%f, %f, %f) | LOST(lost: %u, disorder: %u, repeated: %u)\n",
                  current_frame.sys_timestamp, current_frame.ax, current_frame.ay, current_frame.az,
                  current_frame.gx, current_frame.gy, current_frame.gz,
                  diff * 1e-9, min_diff * 1e-9, max_diff * 1e-9, lost_count, disorder_count, repeated_count);
      last_frame = current_frame;
    }
  }
}

void ImuComponent::recv_func() {
  int ret = 0;
  Bmi08xFrame current_frame;
  while(rclcpp::ok()) {
    ret = bmi08x_get_frame(&bmi08x_device_, &current_frame, imu_use_pool_);
    std::cout << std::flush;
    if (ret != 0) {
      RCLCPP_FATAL(this->get_logger(), "bmi08x_get_frame failed");
      return;
    }
  }
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drobotics::ImuComponent)
