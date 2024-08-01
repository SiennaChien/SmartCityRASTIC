;; Auto-generated. Do not edit!


(when (boundp 'cav_project_2::limo_info)
  (if (not (find-package "CAV_PROJECT_2"))
    (make-package "CAV_PROJECT_2"))
  (shadow 'limo_info (find-package "CAV_PROJECT_2")))
(unless (find-package "CAV_PROJECT_2::LIMO_INFO")
  (make-package "CAV_PROJECT_2::LIMO_INFO"))

(in-package "ROS")
;;//! \htmlinclude limo_info.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cav_project_2::limo_info
  :super ros::object
  :slots (_vel ))

(defmethod cav_project_2::limo_info
  (:init
   (&key
    ((:vel __vel) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _vel __vel)
   self)
  (:vel
   (&rest __vel)
   (if (keywordp (car __vel))
       (send* _vel __vel)
     (progn
       (if __vel (setq _vel (car __vel)))
       _vel)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64 _vel
    (send _vel :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64 _vel
       (send _vel :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64 _vel
     (send _vel :deserialize buf ptr-) (incf ptr- (send _vel :serialization-length))
   ;;
   self)
  )

(setf (get cav_project_2::limo_info :md5sum-) "b2990a44cec4fde7af6f58f2727169f0")
(setf (get cav_project_2::limo_info :datatype-) "cav_project_2/limo_info")
(setf (get cav_project_2::limo_info :definition-)
      "std_msgs/Float64 vel

================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :cav_project_2/limo_info "b2990a44cec4fde7af6f58f2727169f0")


