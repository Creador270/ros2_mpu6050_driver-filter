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

            // Inicialize the time
            previous_time_ = this->now();
        }
    private:
        void FilterImu(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            auto timel = this->now();  // actual time read
            double dt = (timel - previous_time_).seconds();
            previous_time_ = timel;

            //Acelerations
            double ax = msg->linear_acceleration.x;
            double ay = msg->linear_acceleration.y;
            double az = msg->linear_acceleration.z;

            //Gyroscopes
            double gx = msg->angular_velocity.x;
            double gy = msg->angular_velocity.y;

            //Complementary filter
            double rad_to_deg = 180/3.141592654;
            double Acceleration_angle[2] = {0.0, 0.0};
            double Gyroscope_angle[2] = {0.0, 0.0};
            double Total_angle[2] = {0.0, 0.0};
            

            // Acceleration_angle[0] = atan2(ay, az);
            // Acceleration_angle[1] = atan2( -ax, sqrt(pow(ay, 2) + pow(az, 2)));
            /*We implement the euler formula to calculate the angle troug the accelerometer also we change 
             * the aceleration data to LSB/g acordin the sensitive scale factor AFS_SEL=0 usin the value 16384.0
             * from the datasheet*/

            /*Euler formula:
            */

            Acceleration_angle[0] = atan((ay/16384.0)/sqrt(pow((ax/16384.0),2) + pow((az/16384.0),2)))*rad_to_deg;
            Acceleration_angle[1] = atan( -1 * (ax/16384.0)/sqrt(pow((ay /16384.0),2) + pow((az/16384.0),2)))*rad_to_deg;

            /*Seting the giroscope data to LSB/(º/s) acordin the sensitive scale factor FS_SEL=0 usin the value 131
             * from the datasheet*/

            Gyroscope_angle[0] = gx/131.0;
            Gyroscope_angle[1] = gy/131.0;

            /*Aplaying the complementary filter and integrate the angle velocities of the gyroscope, we use a alpa of 0.98*/

            Total_angle[0] = 0.98 *(Total_angle[0] + (Gyroscope_angle[0] * dt)) + (0.02*Acceleration_angle[0]);
            Total_angle[1] = 0.98 *(Total_angle[1] + (Gyroscope_angle[1] * dt)) + (0.02*Acceleration_angle[1]);

            //RCLCPP_INFO(this->get_logger(), "Angles in X: '%f'" , Total_angle[0]);
            //RCLCPP_INFO(this->get_logger(), "Angles in Y: '%f'" , Total_angle[1]);
            //RCLCPP_INFO(this->get_logger(), "Angles in Z: '%s'" , std::to_string(Total_angle[2]));

            // Public the angles in the topic 'angles'
            auto angles = std_msgs::msg::Float64MultiArray();
            angles.data = {Total_angle[0], Total_angle[1]};
            publisher_->publish(angles);

            //sensor_msgs::msg::Imu filtered_msg;
            //filtered_msg.header = msg->header;
        }

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Time previous_time_;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050filter>());
    rclcpp::shutdown();
    return 0;
}