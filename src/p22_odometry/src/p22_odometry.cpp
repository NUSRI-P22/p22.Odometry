#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "serial/serial.h"
#include <cstring>
#include <iostream>

class OdometryNode : public rclcpp::Node
{
public:

    // Replace "/dev/ttyUSB0" with the actual serial port name
    OdometryNode() : Node("p22_odometry"), serial_port_("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000))
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 1000/period_ms);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&OdometryNode::serialCallback, this));
    }

private:
    void serialCallback()
    {
        if (serial_port_.available() > 0)
        {
            nav_msgs::msg::Odometry msg;
            std::string data = serial_port_.readline();
            // data: "x, y, z, Q.x, Q.y, Q.z, Q.w, linear.x, linear.y, linear.z, angular.x, angular.y, angular.z\n"
            int len = data.length();
            float data_value[13];
            int tot = 0;
            std::string tmp = "";
            for(int i = 0; i < len; i++){
                if(data[i] == ' ') continue;
                if(data[i] == ',' || data[i] == '\n'){
                    data_value[tot++] = std::stof(tmp);
                    // std::cout << tmp << std::endl;
                    tmp = "";
                    continue;
                }
                tmp += data[i];
            }
            // std::cout << data << std::endl;
            msg.header.stamp = rclcpp::Clock{}.now();
            msg.header.frame_id = "odom";
            msg.pose.pose.position.x = data_value[0];
            msg.pose.pose.position.y = data_value[1];
            msg.pose.pose.position.z = data_value[2];
            msg.pose.pose.orientation.x = data_value[3];
            msg.pose.pose.orientation.y = data_value[4];
            msg.pose.pose.orientation.z = data_value[5];
            msg.pose.pose.orientation.w = data_value[6];
            msg.twist.twist.linear.x = data_value[7];
            msg.twist.twist.linear.y = data_value[8];
            msg.twist.twist.linear.z = data_value[9];
            msg.twist.twist.angular.x = data_value[10];
            msg.twist.twist.angular.y = data_value[11];
            msg.twist.twist.angular.z = data_value[12];
            publisher_->publish(msg);
        }
    }

    serial::Serial serial_port_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int period_ms = 20; // pubilish period
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
