; Auto-generated. Do not edit!


(cl:in-package serial_comm-msg)


;//! \htmlinclude RadarPointCloud.msg.html

(cl:defclass <RadarPointCloud> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (points
    :reader points
    :initarg :points
    :type (cl:vector serial_comm-msg:RadarCluster)
   :initform (cl:make-array 0 :element-type 'serial_comm-msg:RadarCluster :initial-element (cl:make-instance 'serial_comm-msg:RadarCluster)))
   (num_points
    :reader num_points
    :initarg :num_points
    :type cl:integer
    :initform 0))
)

(cl:defclass RadarPointCloud (<RadarPointCloud>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RadarPointCloud>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RadarPointCloud)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serial_comm-msg:<RadarPointCloud> is deprecated: use serial_comm-msg:RadarPointCloud instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RadarPointCloud>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_comm-msg:header-val is deprecated.  Use serial_comm-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <RadarPointCloud>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_comm-msg:points-val is deprecated.  Use serial_comm-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'num_points-val :lambda-list '(m))
(cl:defmethod num_points-val ((m <RadarPointCloud>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_comm-msg:num_points-val is deprecated.  Use serial_comm-msg:num_points instead.")
  (num_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RadarPointCloud>) ostream)
  "Serializes a message object of type '<RadarPointCloud>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_points)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_points)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_points)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_points)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RadarPointCloud>) istream)
  "Deserializes a message object of type '<RadarPointCloud>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'serial_comm-msg:RadarCluster))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_points)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_points)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_points)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_points)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RadarPointCloud>)))
  "Returns string type for a message object of type '<RadarPointCloud>"
  "serial_comm/RadarPointCloud")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RadarPointCloud)))
  "Returns string type for a message object of type 'RadarPointCloud"
  "serial_comm/RadarPointCloud")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RadarPointCloud>)))
  "Returns md5sum for a message object of type '<RadarPointCloud>"
  "a6e91c51cd4c229b14e7ec1d1022ed42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RadarPointCloud)))
  "Returns md5sum for a message object of type 'RadarPointCloud"
  "a6e91c51cd4c229b14e7ec1d1022ed42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RadarPointCloud>)))
  "Returns full string definition for message of type '<RadarPointCloud>"
  (cl:format cl:nil "# 雷达点云消息~%Header header                          # 标准ROS头部~%serial_comm/RadarCluster[] points      # 雷达点数组~%uint32 num_points                      # 点数量~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: serial_comm/RadarCluster~%# 单个雷达点消息~%float64 x          # X坐标 (m)~%float64 y          # Y坐标 (m) ~%float64 z          # Z坐标 (m)~%float64 velocity   # 速度 (m/s)~%float64 intensity  # 强度/功率值~%float64 range      # 距离 (m)~%float64 azimuth    # 方位角 (度)~%float64 elevation  # 俯仰角 (度)~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RadarPointCloud)))
  "Returns full string definition for message of type 'RadarPointCloud"
  (cl:format cl:nil "# 雷达点云消息~%Header header                          # 标准ROS头部~%serial_comm/RadarCluster[] points      # 雷达点数组~%uint32 num_points                      # 点数量~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: serial_comm/RadarCluster~%# 单个雷达点消息~%float64 x          # X坐标 (m)~%float64 y          # Y坐标 (m) ~%float64 z          # Z坐标 (m)~%float64 velocity   # 速度 (m/s)~%float64 intensity  # 强度/功率值~%float64 range      # 距离 (m)~%float64 azimuth    # 方位角 (度)~%float64 elevation  # 俯仰角 (度)~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RadarPointCloud>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RadarPointCloud>))
  "Converts a ROS message object to a list"
  (cl:list 'RadarPointCloud
    (cl:cons ':header (header msg))
    (cl:cons ':points (points msg))
    (cl:cons ':num_points (num_points msg))
))
