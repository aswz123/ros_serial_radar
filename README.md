# 雷达串口通信包 (Radar Serial Communication Package)

这是一个基于ROS Noetic的雷达数据串口通信包，专门用于处理毫米波雷达的高速串口数据通信和点云生成。

## 功能特性
- **直接集成当前雷达**:发送"AT+START\n"字符串（详见雷达通讯协议）
- **高速串口通信**: 支持2000000波特率的雷达数据传输
- **协议解析**: 自动解析雷达数据帧协议（0x55 0xAA帧头）
- **多种点云格式**: 
  - 标准PCL点云（带velocity字段）
  - 自定义雷达点云（包含完整雷达信息）
- **实时数据处理**: 异步串口数据接收和实时点云发布
- **RViz兼容**: 支持RViz直接可视化
- **数据完整性**: 包含位置、速度、强度、距离、角度等完整信息

## 系统架构

```
雷达设备 → 串口 → serial_node → protocol_parser → 点云话题
                      ↓
               /serial/received
                      ↓
                数据帧解析
                      ↓
            ┌─────────┼─────────┐
            ↓         ↓         ↓
    /radar/pointcloud  │  /radar/pointcloud_custom
    (PointCloud2+V)    │  (完整雷达信息)
                       ↓
              /radar/pointcloud_velocity
                 (备用话题)
```

## 依赖安装

```bash
# 安装基础依赖
sudo apt-get update
sudo apt-get install libserial-dev
sudo apt-get install ros-noetic-serial
sudo apt-get install ros-noetic-pcl-conversions
sudo apt-get install ros-noetic-pcl-ros

# 安装可视化工具（可选）
sudo apt-get install ros-noetic-rviz
```

## 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 1. 启动完整系统

```bash
# 使用launch文件启动完整系统
roslaunch serial_comm radar_system.launch

# 或者分别启动各个节点
roslaunch serial_comm serial_comm.launch port:=/dev/ttyUSB0
rosrun serial_comm protocol_parser
```

### 2. 数据可视化

```bash
# 启动RViz查看点云
rviz

# 在RViz中添加以下显示：
# - PointCloud2: /radar/pointcloud (推荐)
# - PointCloud2: /radar/pointcloud_velocity (备用)
```

### 3. 数据监控

```bash
# 查看原始串口数据
rostopic echo /serial/received

# 查看点云话题
rostopic list | grep radar

# 查看点云数据结构
rostopic echo /radar/pointcloud/fields -n 1

# 查看点云发布频率
rostopic hz /radar/pointcloud

# 查看自定义雷达数据
rostopic echo /radar/pointcloud_custom --noarr -n 1
```

## 话题接口

### 发布的话题

| 话题名称 | 消息类型 | 描述 |
|---------|----------|------|
| `/serial/received` | `std_msgs/String` | 原始16进制串口数据 |
| `/radar/pointcloud` | `sensor_msgs/PointCloud2` | 标准点云+velocity字段 |
| `/radar/pointcloud_velocity` | `sensor_msgs/PointCloud2` | 备用velocity点云 |
| `/radar/pointcloud_custom` | `serial_comm/RadarPointCloud` | 完整雷达信息 |

### 订阅的话题

| 话题名称 | 消息类型 | 描述 |
|---------|----------|------|
| `/serial/send` | `std_msgs/String` | 发送到串口的数据 |

## 点云数据格式

### 主要点云 (`/radar/pointcloud`)
```yaml
字段结构:
  - x: float32        # X坐标 (m)
  - y: float32        # Y坐标 (m)  
  - z: float32        # Z坐标 (m)
  - intensity: float32 # 强度值
  - velocity: float32  # 径向速度 (m/s)
```

### 自定义雷达点云 (`/radar/pointcloud_custom`)
```yaml
字段结构:
  - x, y, z: float64          # 3D坐标 (m)
  - velocity: float64         # 径向速度 (m/s)
  - intensity: float64        # 信号强度
  - range: float64           # 距离 (m)
  - azimuth: float64         # 方位角 (度)
  - elevation: float64       # 俯仰角 (度)
```

## 参数配置

### 串口参数
- `port`: 串口设备路径 (默认: `/dev/ttyUSB0`)
- `baudrate`: 波特率 (默认: `2000000`)
- `timeout`: 超时时间，单位毫秒 (默认: `1000`)

### 雷达参数
- `frame_id`: 坐标系ID (默认: `"radar_frame"`)
- `max_range`: 最大检测距离 (默认: `300m`)
- `min_range`: 最小检测距离 (默认: `0.1m`)

## 数据格式说明

### 雷达协议格式
```
帧头: 0x55 0xAA
数据长度: 2字节
目标数量: 2字节  
CRC校验: 2字节
数据内容: 每个目标9字节
  - 距离索引: 2字节
  - 速度索引: 1字节
  - 方位角索引: 1字节
  - 俯仰角索引: 1字节
  - 功率值: 4字节
```
