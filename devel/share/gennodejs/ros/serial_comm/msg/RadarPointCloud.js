// Auto-generated. Do not edit!

// (in-package serial_comm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RadarCluster = require('./RadarCluster.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RadarPointCloud {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.points = null;
      this.num_points = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = [];
      }
      if (initObj.hasOwnProperty('num_points')) {
        this.num_points = initObj.num_points
      }
      else {
        this.num_points = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RadarPointCloud
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [points]
    // Serialize the length for message field [points]
    bufferOffset = _serializer.uint32(obj.points.length, buffer, bufferOffset);
    obj.points.forEach((val) => {
      bufferOffset = RadarCluster.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [num_points]
    bufferOffset = _serializer.uint32(obj.num_points, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RadarPointCloud
    let len;
    let data = new RadarPointCloud(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [points]
    // Deserialize array length for message field [points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points[i] = RadarCluster.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [num_points]
    data.num_points = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 64 * object.points.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'serial_comm/RadarPointCloud';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a6e91c51cd4c229b14e7ec1d1022ed42';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 雷达点云消息
    Header header                          # 标准ROS头部
    serial_comm/RadarCluster[] points      # 雷达点数组
    uint32 num_points                      # 点数量
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: serial_comm/RadarCluster
    # 单个雷达点消息
    float64 x          # X坐标 (m)
    float64 y          # Y坐标 (m) 
    float64 z          # Z坐标 (m)
    float64 velocity   # 速度 (m/s)
    float64 intensity  # 强度/功率值
    float64 range      # 距离 (m)
    float64 azimuth    # 方位角 (度)
    float64 elevation  # 俯仰角 (度)
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RadarPointCloud(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.points !== undefined) {
      resolved.points = new Array(msg.points.length);
      for (let i = 0; i < resolved.points.length; ++i) {
        resolved.points[i] = RadarCluster.Resolve(msg.points[i]);
      }
    }
    else {
      resolved.points = []
    }

    if (msg.num_points !== undefined) {
      resolved.num_points = msg.num_points;
    }
    else {
      resolved.num_points = 0
    }

    return resolved;
    }
};

module.exports = RadarPointCloud;
