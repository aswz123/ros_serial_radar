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
        points.reserve(targetNum);
        
        size_t index = startIndex;
        int validPoints = 0;
        int totalPoints = 0;
        
        // 计算实际可用数据长度
        size_t availableDataLength = data.size() - startIndex;
        
        // 只检查9字节格式（参考代码使用的格式）
        int bytesPerPoint = 9;
        bool is9ByteFormat = (availableDataLength % 9 == 0) && (availableDataLength / 9 == targetNum);
        
        ROS_INFO("Data analysis: total=%zu, start=%zu, available=%zu", 
                 data.size(), startIndex, availableDataLength);
        ROS_INFO("Target count: %u, expected 9-byte format: %s (calc: %zu/9=%zu)", 
                 targetNum, is9ByteFormat ? "YES" : "NO", 
                 availableDataLength, availableDataLength/9);
        
        if (!is9ByteFormat) {
            // 如果不是标准9字节，尝试计算实际字节数
            if (availableDataLength > 0 && targetNum > 0) {
                int calculatedBytes = availableDataLength / targetNum;
                ROS_WARN("Not standard 9-byte format. Calculated %d bytes per point", calculatedBytes);
                if (calculatedBytes >= 9) {
                    bytesPerPoint = calculatedBytes;
                }
            }
        }
        
        for (int pointCount = 0; pointCount < targetNum && index + bytesPerPoint <= data.size(); pointCount++)
        {
            totalPoints++;
            
            // 解析基本索引值（前5字节，与参考代码一致）
            uint16_t idx1 = data[index] | (data[index + 1] << 8);
            uint8_t idx2 = data[index + 2];
            uint8_t idx3 = data[index + 3];
            uint8_t idx4 = data[index + 4];
            
            // 功率值处理 - 根据参考代码，可能在5-8字节位置
            uint32_t powABS = 0;
            if (bytesPerPoint >= 9) {
                // 按小端格式读取4字节功率值
                powABS = data[index + 5] | (data[index + 6] << 8) | 
                         (data[index + 7] << 16) | (data[index + 8] << 24);
            }
            
            // 详细调试前3个点
            if (pointCount < 3) {
                ROS_INFO("=== Point %d (9-byte format) ===", pointCount);
                
                std::stringstream hex_ss;
                for (int i = 0; i < 9 && index + i < data.size(); i++) {
                    hex_ss << std::hex << std::setw(2) << std::setfill('0') 
                           << (int)data[index + i] << " ";
                }
                ROS_INFO("Raw 9 bytes: %s", hex_ss.str().c_str());
                
                ROS_INFO("idx1=%u, idx2=%u, idx3=%u, idx4=%u", idx1, idx2, idx3, idx4);
                ROS_INFO("Power bytes [5-8]: [%02X %02X %02X %02X] = %u", 
                         data[index+5], data[index+6], data[index+7], data[index+8], powABS);
                
                // 检查功率值是否合理
                if (powABS > 100000000) {  // 超过1亿
                    ROS_WARN("Large intensity detected, trying alternatives:");
                    
                    // 尝试只用前2字节
                    uint16_t powABS_16 = data[index + 5] | (data[index + 6] << 8);
                    ROS_INFO("  16-bit interpretation: %u", powABS_16);
                    
                    // 尝试只用第一个字节
                    uint8_t powABS_8 = data[index + 5];
                    ROS_INFO("  8-bit interpretation: %u", powABS_8);
                }
            }
            
            // 物理值转换（与参考代码一致）
            double RangeRes = 0.02479;
            double _pi = M_PI;
            
            double range = idx1 * RangeRes;
            double velocity = (idx2 - 32) * 0.10416;
            
            double azimuth = 0.0;
            if (idx3 >= 64) {
                azimuth = asin((idx3 - 128) / 64.0) * 180.0 / _pi;
            } else {
                azimuth = asin(idx3 / 64.0) * 180.0 / _pi;
            }
            
            double elevation = 0.0;
            if (idx4 >= 64) {
                elevation = asin((idx4 - 128) / 64.0) * 180.0 / _pi;
            } else {
                elevation = asin(idx4 / 64.0) * 180.0 / _pi;
            }
            
            // 强度值处理 - 如果太大，使用替代方案
            uint32_t finalIntensity = powABS;
            if (powABS > 100000000) {  // 超过1亿，可能解析错误
                finalIntensity = data[index + 5] | (data[index + 6] << 8);  // 只用16位
            }
            
            // 数据有效性检查
            if (range > 0.1 && range < 300 &&
                azimuth >= -90 && azimuth <= 90 &&
                elevation >= -90 && elevation <= 90 &&
                finalIntensity > 0 && finalIntensity < 10000000)  // 限制强度范围
            {
                validPoints++;
                
                // 坐标转换
                double z = range * sin(elevation / 180.0 * _pi);
                double xy = range * cos(elevation / 180.0 * _pi);
                double x = xy * cos(azimuth / 180.0 * _pi);
                double y = xy * sin(azimuth / 180.0 * _pi);
                
                PointData point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.range = range;
                point.velocity = velocity;
                point.azimuth = azimuth;
                point.elevation = elevation;
                point.intensity = finalIntensity;
                
                points.push_back(point);
            }
            
            index += 9;  // 固定使用9字节间隔
        }
        
        ROS_INFO("Parsed %d/%d valid points using 9-byte format", validPoints, totalPoints);
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