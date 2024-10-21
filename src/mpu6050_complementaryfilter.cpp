#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <memory>
#include <math.h> 

//#include "mpu6050driver/mpu6050sensor.hpp"


class MPU6050filter : public rclcpp::Node {
    public:
        MPU6050filter(): Node("filter_node"), time(this->now()) //tiempo inicial
        {
            // Create subscriber
            subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu/data_raw", 10, std::bind(&MPU6050filter::FilterImu, this, std::placeholders::_1));

            // Create publisher
            publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("angles", 10);

            declare_parameter<float>("constant_dt", 0.0);
            get_parameter("constant_dt", constant_dt_);
            RCLCPP_INFO_STREAM(get_logger(), "ElapsedTime dt: " << constant_dt_);

        }
    private:

        void FilterImu(const sensor_msgs::msg::Imu::SharedPtr msg)
        {

            last_time_ = time;
            time = msg->header.stamp;
            //time = this->now();
            //RCLCPP_INFO(this->get_logger(), "Timestamp: '%f'", time.seconds());

            
            // dt = (time - last_time_).seconds();
            // last_time_ = ros::Time::now();

            float dt;
            if (constant_dt_ > 0.0){
                dt = constant_dt_;
                RCLCPP_INFO(this->get_logger(), "Time dt: '%f'", dt);
            }else
            {
                dt = (time - last_time_).seconds();
                // if (time == 0)
                //     RCLCPP_WARN_STREAM_THROTTLE(
                //         5.0,
                //         "The IMU message time stamp is zero, and the parameter "
                //         "constant_dt is not set!"
                //             << " The filter will not update the orientation.");

                RCLCPP_INFO(this->get_logger(), "Time dt: '%f', Time: '%f', Time prev: '%f'", dt, time.seconds(), last_time_.seconds());
            }

            // auto timel = this->now();  // actual time read
            // float dt = (timel - previous_time_).seconds();
            // previous_time_ = timel;

            //Acelerations
            double ax = (msg->linear_acceleration.x) / 9.80665;
            double ay = msg->linear_acceleration.y / 9.80665;
            double az = msg->linear_acceleration.z / 9.80665;

            //Gyroscopes
            double gx = msg->angular_velocity.x;
            double gy = msg->angular_velocity.y;

            //Complementary filter
            float rad_to_deg = 180/M_PI;
            float Acceleration_angle[2] = {0.0, 0.0};
            float Gyroscope_angle[2] = {0.0, 0.0};
            float Total_angle[2] = {0.0, 0.0};
            

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

            Gyroscope_angle[0] = (gx/131.0);
            Gyroscope_angle[1] = (gy/131.0);

            /*Aplaying the complementary filter and integrate the angle velocities of the gyroscope, we use a alpa of 0.98*/

            Total_angle[0] = 0.98 *(Total_angle[0] + (Gyroscope_angle[0] * dt)) + (0.02*Acceleration_angle[0]);
            Total_angle[1] = 0.98 *(Total_angle[1] + (Gyroscope_angle[1] * dt)) + (0.02*Acceleration_angle[1]);

            //RCLCPP_INFO(this->get_logger(), "Angles in X: '%f'" , Total_angle[0]);
            //RCLCPP_INFO(this->get_logger(), "Angles in Y: '%f'" , Total_angle[1]);
            //RCLCPP_INFO(this->get_logger(), "Angles in Z: '%s'" , std::to_string(Total_angle[2]));

            // Public the angles in the topic 'angles'
            auto angles = std_msgs::msg::Float32MultiArray();
            angles.data = {dt, Gyroscope_angle[0]* rad_to_deg, Gyroscope_angle[1]* rad_to_deg, (Gyroscope_angle[0]*rad_to_deg) * dt, (Gyroscope_angle[1]*rad_to_deg) * dt};
            publisher_->publish(angles);

            //sensor_msgs::msg::Imu filtered_msg;
            //filtered_msg.header = msg->header;
        }

        float constant_dt_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        rclcpp::Time time;
        rclcpp::Time last_time_; 
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050filter>());
    rclcpp::shutdown();
    return 0;
}