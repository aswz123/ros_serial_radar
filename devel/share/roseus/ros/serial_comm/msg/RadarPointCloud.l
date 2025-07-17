;; Auto-generated. Do not edit!


(when (boundp 'serial_comm::RadarPointCloud)
  (if (not (find-package "SERIAL_COMM"))
    (make-package "SERIAL_COMM"))
  (shadow 'RadarPointCloud (find-package "SERIAL_COMM")))
(unless (find-package "SERIAL_COMM::RADARPOINTCLOUD")
  (make-package "SERIAL_COMM::RADARPOINTCLOUD"))

(in-package "ROS")
;;//! \htmlinclude RadarPointCloud.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass serial_comm::RadarPointCloud
  :super ros::object
  :slots (_header _points _num_points ))

(defmethod serial_comm::RadarPointCloud
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:points __points) ())
    ((:num_points __num_points) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _points __points)
   (setq _num_points (round __num_points))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:num_points
   (&optional __num_points)
   (if __num_points (setq _num_points __num_points)) _num_points)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; serial_comm/RadarCluster[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; uint32 _num_points
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; serial_comm/RadarCluster[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; uint32 _num_points
       (write-long _num_points s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; serial_comm/RadarCluster[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance serial_comm::RadarCluster :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32 _num_points
     (setq _num_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get serial_comm::RadarPointCloud :md5sum-) "a6e91c51cd4c229b14e7ec1d1022ed42")
(setf (get serial_comm::RadarPointCloud :datatype-) "serial_comm/RadarPointCloud")
(setf (get serial_comm::RadarPointCloud :definition-)
      "# 雷达点云消息
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
")



(provide :serial_comm/RadarPointCloud "a6e91c51cd4c229b14e7ec1d1022ed42")


