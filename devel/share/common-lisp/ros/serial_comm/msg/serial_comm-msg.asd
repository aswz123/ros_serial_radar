
(cl:in-package :asdf)

(defsystem "serial_comm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RadarCluster" :depends-on ("_package_RadarCluster"))
    (:file "_package_RadarCluster" :depends-on ("_package"))
    (:file "RadarPointCloud" :depends-on ("_package_RadarPointCloud"))
    (:file "_package_RadarPointCloud" :depends-on ("_package"))
  ))