;; Auto-generated. Do not edit!


(when (boundp 'serial_comm::RadarCluster)
  (if (not (find-package "SERIAL_COMM"))
    (make-package "SERIAL_COMM"))
  (shadow 'RadarCluster (find-package "SERIAL_COMM")))
(unless (find-package "SERIAL_COMM::RADARCLUSTER")
  (make-package "SERIAL_COMM::RADARCLUSTER"))

(in-package "ROS")
;;//! \htmlinclude RadarCluster.msg.html


(defclass serial_comm::RadarCluster
  :super ros::object
  :slots (_x _y _z _velocity _intensity _range _azimuth _elevation ))

(defmethod serial_comm::RadarCluster
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:velocity __velocity) 0.0)
    ((:intensity __intensity) 0.0)
    ((:range __range) 0.0)
    ((:azimuth __azimuth) 0.0)
    ((:elevation __elevation) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _velocity (float __velocity))
   (setq _intensity (float __intensity))
   (setq _range (float __range))
   (setq _azimuth (float __azimuth))
   (setq _elevation (float __elevation))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:intensity
   (&optional __intensity)
   (if __intensity (setq _intensity __intensity)) _intensity)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:azimuth
   (&optional __azimuth)
   (if __azimuth (setq _azimuth __azimuth)) _azimuth)
  (:elevation
   (&optional __elevation)
   (if __elevation (setq _elevation __elevation)) _elevation)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _velocity
    8
    ;; float64 _intensity
    8
    ;; float64 _range
    8
    ;; float64 _azimuth
    8
    ;; float64 _elevation
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _intensity
       (sys::poke _intensity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _range
       (sys::poke _range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _azimuth
       (sys::poke _azimuth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _elevation
       (sys::poke _elevation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _intensity
     (setq _intensity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _range
     (setq _range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _azimuth
     (setq _azimuth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _elevation
     (setq _elevation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get serial_comm::RadarCluster :md5sum-) "0840e87e925ffcb716d5163155ce63c6")
(setf (get serial_comm::RadarCluster :datatype-) "serial_comm/RadarCluster")
(setf (get serial_comm::RadarCluster :definition-)
      "# 单个雷达点消息
float64 x          # X坐标 (m)
float64 y          # Y坐标 (m) 
float64 z          # Z坐标 (m)
float64 velocity   # 速度 (m/s)
float64 intensity  # 强度/功率值
float64 range      # 距离 (m)
float64 azimuth    # 方位角 (度)
float64 elevation  # 俯仰角 (度)
")



(provide :serial_comm/RadarCluster "0840e87e925ffcb716d5163155ce63c6")


