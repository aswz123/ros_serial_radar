#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cmath>

class ProtocolParser
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pointcloud_pub_;
    ros::Subscriber hex_data_sub_;
    
    struct PointData
    {
        double x, y, z;
        double range;
        double velocity;
        double azimuth;
        double elevation;
        uint32_t intensity;
    };

public:
    ProtocolParser() : nh_("~")
    {
        // 初始化发布者和订阅者
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar/pointcloud", 10);
        hex_data_sub_ = nh_.subscribe("/serial/received", 10, &ProtocolParser::hexDataCallback, this);
        
        ROS_INFO("Protocol Parser initialized");
    }
    
    void hexDataCallback(const std_msgs::String::ConstPtr& msg)
    {
        parseHexString(msg->data);
    }
    
    void parseHexString(const std::string& hexStr)
    {
        std::vector<uint8_t> data = hexStringToBytes(hexStr);
        
        if (data.size() < 12) // 最小帧长度检查
        {
            return;
        }
        
        // 检查帧头
        if (data[0] != 0x55 || data[1] != 0xAA)
        {
            return;
        }
        
        // 解析帧长度 (小端序)
        uint32_t frameLength = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
        
        // 解析时间间隔
        uint16_t timeInterval = data[6] | (data[7] << 8);
        
        // 解析TLV数量
        uint8_t numTLVs = data[8];
        
        // 解析类型
        uint8_t type = data[9];
        
        // 解析目标数量
        uint16_t targetNum = data[10] | (data[11] << 8);
        
        ROS_INFO("解析到 %d 个目标", targetNum);
        
        // 解析TLV数据并发布点云
        if (data.size() > 12)
        {
            std::vector<PointData> points = parseTLVData(data, 12, targetNum);
            publishPointCloud(points);
        }
    }

private:
    std::vector<uint8_t> hexStringToBytes(const std::string& hexStr)
    {
        std::vector<uint8_t> bytes;
        std::stringstream ss(hexStr);
        std::string hex;
        
        while (ss >> hex)
        {
            if (hex.length() == 2)
            {
                uint8_t byte = static_cast<uint8_t>(std::stoul(hex, nullptr, 16));
                bytes.push_back(byte);
            }
        }
        
        return bytes;
    }
    
    std::vector<PointData> parseTLVData(const std::vector<uint8_t>& data, size_t startIndex, uint16_t targetNum)
    {
        std::vector<PointData> points;
        size_t index = startIndex;
        
        while (index < data.size() && index + 4 < data.size())
        {
            // 解析TLV头部
            uint8_t type = data[index];
            uint8_t length = data[index + 1];
            
            if (type == 1) // 点云数据
            {
                std::vector<PointData> tlv_points = parsePointCloudData(data, index + 2, length, targetNum);
                points.insert(points.end(), tlv_points.begin(), tlv_points.end());
            }
            
            index += 2 + length; // 移动到下一个TLV
        }
        
        return points;
    }
    
    std::vector<PointData> parsePointCloudData(const std::vector<uint8_t>& data, size_t startIndex, uint8_t length, uint16_t targetNum)
    {
        std::vector<PointData> points;
        size_t index = startIndex;
        int pointCount = 0;
        
        // 每个点包含: idx1(2B) + idx2(2B) + idx3(2B) + idx4(2B) + powABS(4B) = 12字节
        while (index + 12 <= startIndex + length && pointCount < targetNum)
        {
            // 解析距离和角度索引
            uint16_t idx1 = data[index] | (data[index + 1] << 8);
            uint16_t idx2 = data[index + 2] | (data[index + 3] << 8);
            uint16_t idx3 = data[index + 4] | (data[index + 5] << 8);
            uint16_t idx4 = data[index + 6] | (data[index + 7] << 8);
            
            // 解析功率值
            uint32_t powABS = data[index + 8] | (data[index + 9] << 8) | 
                             (data[index + 10] << 16) | (data[index + 11] << 24);
            
            // 计算实际物理值
            double range = idx1 * 0.05; // 距离 (m)
            double velocity = (idx2 - 32) * 0.10416; // 速度 (m/s)
            double azimuth = (idx3 - 127) * 180.0 / M_PI; // 方位角 (度)
            double elevation = (idx4 - 127) * 180.0 / M_PI; // 俯仰角 (度)
            
            // 将角度转换为弧度
            double azimuth_rad = azimuth * M_PI / 180.0;
            double elevation_rad = elevation * M_PI / 180.0;
            
            // 球坐标转直角坐标
            double x = range * cos(elevation_rad) * cos(azimuth_rad);
            double y = range * cos(elevation_rad) * sin(azimuth_rad);
            double z = range * sin(elevation_rad);
            
            // 创建点数据
            PointData point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.range = range;
            point.velocity = velocity;
            point.azimuth = azimuth;
            point.elevation = elevation;
            point.intensity = powABS;
            
            points.push_back(point);
            
            ROS_INFO("点 %d: 距离=%.2fm, 速度=%.2fm/s, 方位角=%.1f°, 俯仰角=%.1f°, XYZ=(%.2f,%.2f,%.2f)", 
                     pointCount + 1, range, velocity, azimuth, elevation, x, y, z);
            
            index += 12;
            pointCount++;
        }
        
        return points;
    }
    
    void publishPointCloud(const std::vector<PointData>& points)
    {
        if (points.empty())
        {
            return;
        }
        
        // 创建PCL点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (const auto& point : points)
        {
            pcl::PointXYZI pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;
            pcl_point.intensity = point.intensity;
            
            cloud->points.push_back(pcl_point);
        }
        
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        
        // 转换为ROS消息
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "radar_frame";
        
        // 发布点云
        pointcloud_pub_.publish(cloud_msg);
        
        ROS_INFO("发布了包含 %zu 个点的点云", points.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "protocol_parser");
    
    ProtocolParser parser;
    
    ros::spin();
    
    return 0;
}