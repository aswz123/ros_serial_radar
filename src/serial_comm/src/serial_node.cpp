#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>

class SerialNode
{
private:
    ros::NodeHandle nh_;
    ros::Publisher serial_pub_;
    ros::Subscriber serial_sub_;
    serial::Serial ser_;
    
    std::string port_;
    int baudrate_;
    int timeout_;
    
public:
    SerialNode() : nh_("~")
    {
        // 获取参数
        nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        nh_.param<int>("baudrate", baudrate_, 2000000);
        nh_.param<int>("timeout", timeout_, 1000);
        
        // 初始化发布者和订阅者
        serial_pub_ = nh_.advertise<std_msgs::String>("/serial/received", 10);
        serial_sub_ = nh_.subscribe("/serial/send", 10, &SerialNode::serialCallback, this);
        
        // 配置串口
        try
        {
            ser_.setPort(port_);
            ser_.setBaudrate(baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
            ser_.setTimeout(to);
            ser_.open();
            
            ROS_INFO("Serial port opened successfully");
            ROS_INFO("Port: %s", port_.c_str());
            ROS_INFO("Baudrate: %d", baudrate_);
            ROS_INFO("Timeout: %d ms", timeout_);
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR("Unable to open port %s: %s", port_.c_str(), e.what());
            return;
        }
        
        if(ser_.isOpen())
        {
            ROS_INFO("Serial Port initialized");
        }
        else
        {
            ROS_ERROR("Serial port failed to open");
        }
    }
    
    ~SerialNode()
    {
        if(ser_.isOpen())
        {
            ser_.close();
            ROS_INFO("Serial port closed");
        }
    }
    
    void serialCallback(const std_msgs::String::ConstPtr& msg)
    {
        if(ser_.isOpen())
        {
            try
            {
                ser_.write(msg->data);
                ROS_INFO("Sent: %s", msg->data.c_str());
            }
            catch(serial::IOException& e)
            {
                ROS_ERROR("Error writing to serial port: %s", e.what());
            }
        }
    }
    
    void readSerial()
    {
        if(ser_.isOpen() && ser_.available())
        {
            try
            {
                std::string result = ser_.read(ser_.available());
                if(!result.empty())
                {
                    std_msgs::String msg;
                    msg.data = result;
                    serial_pub_.publish(msg);
                    ROS_INFO("Received: %s", result.c_str());
                }
            }
            catch(serial::IOException& e)
            {
                ROS_ERROR("Error reading from serial port: %s", e.what());
            }
        }
    }
    
    void spin()
    {
        ros::Rate rate(1000); // 1000 Hz for high-speed communication
        
        while(ros::ok())
        {
            readSerial();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_node");
    
    SerialNode serial_node;
    serial_node.spin();
    
    return 0;
}
