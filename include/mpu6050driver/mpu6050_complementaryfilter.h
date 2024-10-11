#ifndef MPU6050_FILTER_H
#define MPU6050_FILTER_H

#include "mpu6050driver/mpu6050sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
class MPU6050Dfilter : public rclcpp::Node {
 public:
  MPU6050Dfilter();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  //rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  void FilterImu();
};

#endif  // MPU6050_FILTER_H