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
            
            // 等待串口稳定并清空缓冲区
            ros::Duration(0.5).sleep(); // 等待500ms让串口稳定
            
            // 清空输入缓冲区
            try
            {
                if(ser_.available() > 0)
                {
                    std::string dummy = ser_.read(ser_.available());
                    ROS_INFO("Cleared %zu bytes from input buffer", dummy.length());
                }
            }
            catch(serial::IOException& e)
            {
                ROS_WARN("Error clearing buffer: %s", e.what());
            }
            
            // 再等待一下确保缓冲区清空
            ros::Duration(0.2).sleep();
            
            // 发送启动命令
            try
            {
                std::string start_cmd = "AT+START\n";
                ser_.write(start_cmd);
                ROS_INFO("Sent startup command: %s", start_cmd.c_str());
                
                // 发送命令后稍等一下
                ros::Duration(0.1).sleep();
            }
            catch(serial::IOException& e)
            {
                ROS_ERROR("Error sending startup command: %s", e.what());
            }
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
                    // 转换为16进制字符串
                    std::stringstream hex_stream;
                    for(unsigned char c : result)
                    {
                        hex_stream << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << (int)c << " ";
                    }
                    std::string hex_string = hex_stream.str();
                    
                    std_msgs::String msg;
                    msg.data = hex_string;
                    serial_pub_.publish(msg);
                    ROS_INFO("Received (HEX): %s", hex_string.c_str());
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
