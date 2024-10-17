#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <memory>
#include <math.h> 

//#include "mpu6050driver/mpu6050sensor.hpp"


class MPU6050filter : public rclcpp::Node {
    public:
        MPU6050filter(): Node("filter_node")
        {
            // Create subscriber
            subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu", 10, std::bind(&MPU6050filter::FilterImu, this, std::placeholders::_1));

            // Create publisher
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("angles", 10);
        }
    private:
        void FilterImu(const sensor_msgs::msg::Imu::SharedPtr msg)
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
            double rad_to_deg = 180/3.141592654;
            double Acceleration_angle[3] = {0.0, 0.0, 0.0};
            double Total_angle[3] = {0.0, 0.0, 0.0};

            Acceleration_angle[0] = atan((ay / 16384.0) / sqrt(pow(ax/16384.0, 2) + pow(az/16384.0, 2))) * rad_to_deg;
            Acceleration_angle[1] = atan(-1 * (ax / 16384.0) / sqrt(pow(ay/16384.0, 2) + pow(az/16384.0, 2))) * rad_to_deg;
            Acceleration_angle[2] = atan((az / 16384.0) / sqrt(pow(ax/16384.0, 2) + pow(ay/16384.0, 2))) * rad_to_deg;

            Total_angle[0] = 0.98 *(Total_angle[0] + (gx / 131.0)*dt) + 0.02 * Acceleration_angle[0];
            Total_angle[1] = 0.98 *(Total_angle[1] + (gy / 131.0)*dt) + 0.02 * Acceleration_angle[1];
            Total_angle[2] = 0.98 *(Total_angle[2] + (gz / 131.0)*dt) + 0.02 * Acceleration_angle[2];

            RCLCPP_INFO(this->get_logger(), "Angles in X: '%f'" , Total_angle[0]);
            RCLCPP_INFO(this->get_logger(), "Angles in Y: '%f'" , Total_angle[1]);
            //RCLCPP_INFO(this->get_logger(), "Angles in Z: '%s'" , std::to_string(Total_angle[2]));

            // Publicar los ángulos en el tópico 'angles'
            auto angles = std_msgs::msg::Float64MultiArray();
            angles.data = {Total_angle[0], Total_angle[1]};
            publisher_->publish(angles);

            //sensor_msgs::msg::Imu filtered_msg;
            //filtered_msg.header = msg->header;
        }

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050filter>());
    rclcpp::shutdown();
    return 0;
}