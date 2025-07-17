#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/String.h>
#include <serial_comm/RadarCluster.h>
#include <serial_comm/RadarPointCloud.h>
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
    ros::Publisher radar_pointcloud_pub_;  // 新的自定义消息发布者
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

    std::vector<uint8_t> hexStringToBytes(const std::string& hexStr)
    {
        std::vector<uint8_t> bytes;
        std::stringstream ss(hexStr);
        std::string hex;
        
        while (ss >> hex)
        {
            if (hex.length() == 2)
            {
                try
                {
                    uint8_t byte = static_cast<uint8_t>(std::stoul(hex, nullptr, 16));
                    bytes.push_back(byte);
                }
                catch(const std::exception& e)
                {
                    ROS_WARN("Cannot parse hex string: %s", hex.c_str());
                }
            }
        }
        
        return bytes;
    }

public:
    ProtocolParser() : nh_("~")
    {
        // 初始化发布者和订阅者
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar/pointcloud", 10);
        radar_pointcloud_pub_ = nh_.advertise<serial_comm::RadarPointCloud>("/radar/pointcloud_custom", 10);
        hex_data_sub_ = nh_.subscribe("/serial/received", 100, &ProtocolParser::hexDataCallback, this); // 增大队列
        
        ROS_INFO("Protocol Parser initialized with custom messages");
        
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
        
        if (data.empty()) return;
        
        // Add new data to buffer
        data_buffer_.insert(data_buffer_.end(), data.begin(), data.end());
        
        // 只在调试模式下输出详细信息
        // ROS_DEBUG("Received %zu bytes, buffer total: %zu bytes", data.size(), data_buffer_.size());
        
        // Process complete frames in buffer
        processBuffer();
    }

    void processBuffer()
    {
        while (data_buffer_.size() >= 12)
        {
            // Find frame header 0x55 0xAA
            size_t header_pos = SIZE_MAX;
            
            for (size_t i = 0; i <= data_buffer_.size() - 2; i++)
            {
                if (data_buffer_[i] == 0x55 && data_buffer_[i + 1] == 0xAA)
                {
                    header_pos = i;
                    break;
                }
            }
            
            if (header_pos == SIZE_MAX)
            {
                // No frame header found, keep last byte (might be start of 0x55)
                if (data_buffer_.size() > 1)
                {
                    if (data_buffer_.back() == 0x55)
                    {
                        uint8_t last_byte = data_buffer_.back();
                        data_buffer_.clear();
                        data_buffer_.push_back(last_byte);
                    }
                    else
                    {
                        data_buffer_.clear();
                    }
                }
                break;
            }
            
            // Remove invalid data before frame header
            if (header_pos > 0)
            {
                data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + header_pos);
            }
            
            // Check if we have enough data to read frame length
            if (data_buffer_.size() < 6)
            {
                break; // Wait for more data
            }
            
            // Parse frame length (bytes 2-5, little endian)
            uint32_t frameLength = data_buffer_[2] | (data_buffer_[3] << 8) | 
                                  (data_buffer_[4] << 16) | (data_buffer_[5] << 24);
            
            // 调整帧长度限制以适应毫米波雷达的大数据帧
            if (frameLength > 10000 || frameLength < 12)  // 最大允许10KB的帧
            {
                ROS_WARN("Abnormal frame length: %u, removing header and searching again", frameLength);
                data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + 2);
                continue;
            }
            
            // Check if we have complete frame
            if (data_buffer_.size() < frameLength)
            {
                break; // Wait for more data
            }
            
            // Extract complete frame
            std::vector<uint8_t> frame(data_buffer_.begin(), data_buffer_.begin() + frameLength);
            data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + frameLength);
            
            // Parse this frame
            parseFrame(frame);
        }
        
        // 增大缓冲区限制以处理大数据帧
        if (data_buffer_.size() > 15000) // 15KB缓冲区
        {
            ROS_WARN("Buffer too large (%zu bytes), clearing", data_buffer_.size());
            data_buffer_.clear();
        }
    }
    
    void parseFrame(const std::vector<uint8_t>& data)
    {
        if (data.size() < 12)
        {
            ROS_WARN("Frame length insufficient: %zu", data.size());
            return;
        }
        
        // Verify frame header
        if (data[0] != 0x55 || data[1] != 0xAA)
        {
            ROS_WARN("Frame header verification failed");
            return;
        }
        
        // Parse frame components
        uint32_t frameLength = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
        uint16_t timeInterval = data[6] | (data[7] << 8);
        uint8_t numTLVs = data[8];
        uint8_t type = data[9];
        uint16_t targetNum = data[10] | (data[11] << 8);
        
        // 输出帧信息（毫米波雷达可能有很多目标）
        ROS_INFO("Frame: %u bytes, %u targets", frameLength, targetNum);
        
        // 调整目标数量限制以适应毫米波雷达
        if (targetNum > 0 && targetNum < 1000) // 最大允许1000个目标点
        {
            // Parse point cloud data directly
            std::vector<PointData> points = parseDirectPointData(data, 12, targetNum);
            
            // 发布点云
            publishPointCloud(points);
        }
        else
        {
            // 如果目标数异常，发布空点云保持连续性
            std::vector<PointData> empty_points;
            publishPointCloud(empty_points);
            ROS_WARN("Abnormal target count: %u, publishing empty pointcloud", targetNum);
        }
    }

    // 新增：直接解析点数据的函数
    std::vector<PointData> parseDirectPointData(const std::vector<uint8_t>& data, size_t startIndex, uint16_t targetNum)
    {
        std::vector<PointData> points;
        points.reserve(targetNum); // 预分配内存提高性能
        
        size_t index = startIndex;
        int validPoints = 0;
        int totalPoints = 0;
        
        // 每个点12字节
        for (int pointCount = 0; pointCount < targetNum && index + 12 <= data.size(); pointCount++)
        {
            totalPoints++;
            
            // 解析索引值
            uint16_t idx1 = data[index] | (data[index + 1] << 8);
            uint16_t idx2 = data[index + 2] | (data[index + 3] << 8);
            uint16_t idx3 = data[index + 4] | (data[index + 5] << 8);
            uint16_t idx4 = data[index + 6] | (data[index + 7] << 8);
            
            // 解析功率值
            uint32_t powABS = data[index + 8] | (data[index + 9] << 8) | 
                             (data[index + 10] << 16) | (data[index + 11] << 24);
            
            // 物理值转换
            double range = idx1 * 0.05; // 距离 (m)
            double velocity = (int16_t)(idx2 - 32768) * 0.10416; // 速度 (m/s)
            double azimuth = (int16_t)(idx3 - 32768) * 180.0 / 32768.0; // 方位角 (度)
            double elevation = (int16_t)(idx4 - 32768) * 90.0 / 32768.0; // 俯仰角 (度)
            
            // 数据有效性检查（针对毫米波雷达调整）
            if (range > 0.1 && range < 300 &&  // 毫米波雷达通常探测距离较远
                azimuth >= -180 && azimuth <= 180 && 
                elevation >= -90 && elevation <= 90 &&
                powABS > 0) // 确保功率值有效
            {
                validPoints++;
                
                // 转换为弧度
                double azimuth_rad = azimuth * M_PI / 180.0;
                double elevation_rad = elevation * M_PI / 180.0;
                
                // 球坐标转直角坐标
                double x = range * cos(elevation_rad) * cos(azimuth_rad);
                double y = range * cos(elevation_rad) * sin(azimuth_rad);
                double z = range * sin(elevation_rad);
                
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
            }
            
            index += 12;
        }
        
        // 只在点数较少时输出详细统计，避免大量输出
        if (totalPoints > 0)
        {
            if (totalPoints <= 50)
            {
                ROS_INFO("Parsed %d/%d valid points (%.1f%%)", validPoints, totalPoints, 
                         100.0 * validPoints / totalPoints);
            }
            else
            {
                // 大量点时只输出简要信息
                ROS_INFO("Parsed %d/%d valid points", validPoints, totalPoints);
            }
        }
        
        return points;
    }
    
    void publishPointCloud(const std::vector<PointData>& points)
    {
        // 创建PCL点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->reserve(points.size()); // 预分配内存
        
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
        
        // 发布自定义RadarPointCloud消息
        publishRadarPointCloud(points);
        
        // 简化输出信息
        if (points.empty())
        {
            ROS_DEBUG("Published empty pointcloud");
        }
        else
        {
            ROS_INFO("Published pointcloud: %zu points", points.size());
        }
    }
    
    void publishRadarPointCloud(const std::vector<PointData>& points)
    {
        serial_comm::RadarPointCloud radar_msg;
        
        // 设置头部信息
        radar_msg.header.stamp = ros::Time::now();
        radar_msg.header.frame_id = "radar_frame";
        radar_msg.num_points = points.size();
        
        // 转换点数据
        for (const auto& point : points)
        {
            serial_comm::RadarCluster radar_point;
            radar_point.x = point.x;
            radar_point.y = point.y;
            radar_point.z = point.z;
            radar_point.velocity = point.velocity;
            radar_point.intensity = point.intensity;
            radar_point.range = point.range;
            radar_point.azimuth = point.azimuth;
            radar_point.elevation = point.elevation;
            
            radar_msg.points.push_back(radar_point);
        }
        
        // 发布自定义消息
        radar_pointcloud_pub_.publish(radar_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "protocol_parser");
    
    ProtocolParser parser;
    
    ros::spin();
    
    return 0;
}