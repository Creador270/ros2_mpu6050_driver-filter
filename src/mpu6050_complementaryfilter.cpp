#include "mpu6050driver/mpu6050_complementaryfilter.h"

#include <memory>
#include <math.h> 

MPU6050Dfilter::MPU6050Dfilter(): Node("filter_node"){

    // Create subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&MPU6050Dfilter::handleImu, this, _1));

    // Create publisher
    //publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("Angles", 10);
    }


void MPU6050Dfilter::FilterImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    //Acelerations
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    //Gyroscopes
    double gx = msg->angular_velocity.x;
    double gy = msg->angular_velocity.y;
    double gz = msg->angular_velocity.z;

    //Complementary filter
    double dt = 0.01;
    float rad_to_deg = 180/3.141592654;
    float Acceleration_angle[3];
    float Total_angle[3];

    Acceleration_angle[0] = atan((ay / 16384.0) / sqrt(pow(ax/16384.0, 2) + pow(az/16384.0, 2))) * rad_to_deg;
    Acceleration_angle[1] = atan(-1 * (ax / 16384.0) / sqrt(pow(ay/16384.0, 2) + pow(az/16384.0, 2))) * rad_to_deg;
    //Acceleration_angle[2] = atan((az / 16384.0) / sqrt(pow(ax/16384.0, 2) + pow(ay/16384.0, 2))) * rad_to_deg;

    Total_angle[0] = 0.98 *(Total_angle[0] + (gx / 131.0)) + 0.02 * Acceleration_angle[0];
    Total_angle[1] = 0.98 *(Total_angle[1] + (gy / 131.0)) + 0.02 * Acceleration_angle[1];
    //Total_angle[2] = 0.98 *(Total_angle[2] + (gz / 131.0)) + 0.02 * Acceleration_angle[2];

    RCLCPP_INFO(this->get_logger(), "Angles in X: '%s'" , std::to_string(Total_angle[0]));
    RCLCPP_INFO(this->get_logger(), "Angles in Y: '%s'" , std::to_string(Total_angle[1]));
    //RCLCPP_INFO(this->get_logger(), "Angles in Z: '%s'" , std::to_string(Total_angle[2]));

}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050Dfilter>());
    rclcpp::shutdown();
    return 0;
}