# 串口通信包 (Serial Communication Package)

这是一个基于ROS Noetic的高速串口通信包，支持2000000波特率的串口发送与接收。

## 功能特性

- 支持高波特率通信（默认2000000）
- 异步串口数据接收
- ROS话题接口便于集成
- 可配置的串口参数
- 错误处理和重连机制

## 依赖安装

```bash
# 安装libserial库
sudo apt-get update
sudo apt-get install libserial-dev

# 如果使用的是ros-serial库，可以安装：
sudo apt-get install ros-noetic-serial
```

## 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 1. 启动串口节点

```bash
# 使用默认参数启动
roslaunch serial_comm serial_comm.launch

# 或者指定串口设备
roslaunch serial_comm serial_comm.launch port:=/dev/ttyUSB1

# 或者使用rosrun直接启动
rosrun serial_comm serial_node _port:=/dev/ttyUSB0 _baudrate:=2000000
```

### 2. 发送数据

```bash
# 使用测试脚本
rosrun serial_comm serial_sender.py

# 或者直接发布消息
rostopic pub /serial/send std_msgs/String "data: 'Hello World'"
```

### 3. 接收数据

```bash
# 监听接收到的数据
rostopic echo /serial/received
```

## 话题接口

### 发布的话题
- `/serial/received` (std_msgs/String): 从串口接收到的数据

### 订阅的话题
- `/serial/send` (std_msgs/String): 要发送到串口的数据

## 参数配置

- `port`: 串口设备路径 (默认: /dev/ttyUSB0)
- `baudrate`: 波特率 (默认: 2000000)
- `timeout`: 超时时间，单位毫秒 (默认: 1000)

## 注意事项

1. 确保串口设备有正确的权限：
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # 或者将用户添加到dialout组
   sudo usermod -a -G dialout $USER
   ```

2. 高波特率通信需要确保硬件支持，某些USB转串口设备可能不支持2000000波特率。

3. 如果遇到权限问题，可能需要重新登录或重启。

## 故障排除

- 如果无法打开串口，检查设备是否存在且有正确权限
- 如果编译失败，确保已安装libserial-dev
- 如果通信不稳定，尝试降低波特率或检查硬件连接
