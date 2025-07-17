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
    
    std::vector<uint8_t> data_buffer_; // 数据缓冲区
    
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
        hex_data_sub_ = nh_.subscribe("/serial/received", 100, &ProtocolParser::hexDataCallback, this); // 增大队列
        
        ROS_INFO("Protocol Parser initialized");
        
        // 给串口节点一些时间来初始化
        ros::Duration(1.0).sleep();
    }
    
    void hexDataCallback(const std_msgs::String::ConstPtr& msg)
    {
        parseHexString(msg->data);
    }
    
    void parseHexString(const std::string& hexStr)
    {
        std::vector<uint8_t> data = hexStringToBytes(hexStr);
        
        // 将新数据添加到缓冲区
        data_buffer_.insert(data_buffer_.end(), data.begin(), data.end());
        
        // 处理缓冲区中的完整帧
        processBuffer();
    }
    
    void processBuffer()
    {
        while (data_buffer_.size() >= 12) // 最小帧长度检查
        {
            // 寻找帧头
            auto it = std::find_if(data_buffer_.begin(), data_buffer_.end() - 1, 
                [this](uint8_t byte) {
                    auto next_it = std::next(&byte - &data_buffer_[0] + data_buffer_.begin());
                    return byte == 0x55 && next_it != data_buffer_.end() && *next_it == 0xAA;
                });
            
            if (it == data_buffer_.end() - 1) {
                // 没有找到完整的帧头，保留最后一个字节
                if (data_buffer_.size() > 1) {
                    data_buffer_.erase(data_buffer_.begin(), data_buffer_.end() - 1);
                }
                break;
            }
            
            // 移除帧头之前的数据
            size_t header_pos = std::distance(data_buffer_.begin(), it);
            if (header_pos > 0) {
                data_buffer_.erase(data_buffer_.begin(), it);
            }
            
            // 检查是否有足够的数据读取帧长度
            if (data_buffer_.size() < 12) {
                break;
            }
            
            // 检查帧头
            if (data_buffer_[0] != 0x55 || data_buffer_[1] != 0xAA) {
                data_buffer_.erase(data_buffer_.begin()); // 移除错误的字节
                continue;
            }
            
            // 解析帧长度 (小端序)
            uint32_t frameLength = data_buffer_[2] | (data_buffer_[3] << 8) | 
                                 (data_buffer_[4] << 16) | (data_buffer_[5] << 24);
            
            // 检查帧长度是否合理（避免过大的帧）
            if (frameLength > 1000 || frameLength < 12) {
                data_buffer_.erase(data_buffer_.begin()); // 移除错误的字节
                continue;
            }
            
            // 检查是否有完整的帧
            if (data_buffer_.size() < frameLength) {
                break; // 等待更多数据
            }
            
            // 提取完整的帧
            std::vector<uint8_t> frame(data_buffer_.begin(), data_buffer_.begin() + frameLength);
            data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + frameLength);
            
            // 解析这个帧
            parseFrame(frame);
        }
        
        // 防止缓冲区过大
        if (data_buffer_.size() > 2000) {
            data_buffer_.clear();
            ROS_WARN("清空过大的数据缓冲区");
        }
    }
    
    void parseFrame(const std::vector<uint8_t>& data)
    {
        if (data.size() < 12) return;
        
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
        if (data.size() > 12 && targetNum > 0)
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