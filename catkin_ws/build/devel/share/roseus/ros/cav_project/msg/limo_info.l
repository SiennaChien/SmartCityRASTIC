;; Auto-generated. Do not edit!


(when (boundp 'cav_project::limo_info)
  (if (not (find-package "CAV_PROJECT"))
    (make-package "CAV_PROJECT"))
  (shadow 'limo_info (find-package "CAV_PROJECT")))
(unless (find-package "CAV_PROJECT::LIMO_INFO")
  (make-package "CAV_PROJECT::LIMO_INFO"))

(in-package "ROS")
;;//! \htmlinclude limo_info.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cav_project::limo_info
  :super ros::object
  :slots (_ID _x _y _vel _path _d1 _d2 _origin_dist ))

(defmethod cav_project::limo_info
  (:init
   (&key
    ((:ID __ID) (instance std_msgs::Int32 :init))
    ((:x __x) (instance std_msgs::Float64 :init))
    ((:y __y) (instance std_msgs::Float64 :init))
    ((:vel __vel) (instance std_msgs::Float64 :init))
    ((:path __path) (instance std_msgs::String :init))
    ((:d1 __d1) (instance std_msgs::Float64 :init))
    ((:d2 __d2) (instance std_msgs::Float64 :init))
    ((:origin_dist __origin_dist) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _ID __ID)
   (setq _x __x)
   (setq _y __y)
   (setq _vel __vel)
   (setq _path __path)
   (setq _d1 __d1)
   (setq _d2 __d2)
   (setq _origin_dist __origin_dist)
   self)
  (:ID
   (&rest __ID)
   (if (keywordp (car __ID))
       (send* _ID __ID)
     (progn
       (if __ID (setq _ID (car __ID)))
       _ID)))
  (:x
   (&rest __x)
   (if (keywordp (car __x))
       (send* _x __x)
     (progn
       (if __x (setq _x (car __x)))
       _x)))
  (:y
   (&rest __y)
   (if (keywordp (car __y))
       (send* _y __y)
     (progn
       (if __y (setq _y (car __y)))
       _y)))
  (:vel
   (&rest __vel)
   (if (keywordp (car __vel))
       (send* _vel __vel)
     (progn
       (if __vel (setq _vel (car __vel)))
       _vel)))
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:d1
   (&rest __d1)
   (if (keywordp (car __d1))
       (send* _d1 __d1)
     (progn
       (if __d1 (setq _d1 (car __d1)))
       _d1)))
  (:d2
   (&rest __d2)
   (if (keywordp (car __d2))
       (send* _d2 __d2)
     (progn
       (if __d2 (setq _d2 (car __d2)))
       _d2)))
  (:origin_dist
   (&rest __origin_dist)
   (if (keywordp (car __origin_dist))
       (send* _origin_dist __origin_dist)
     (progn
       (if __origin_dist (setq _origin_dist (car __origin_dist)))
       _origin_dist)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Int32 _ID
    (send _ID :serialization-length)
    ;; std_msgs/Float64 _x
    (send _x :serialization-length)
    ;; std_msgs/Float64 _y
    (send _y :serialization-length)
    ;; std_msgs/Float64 _vel
    (send _vel :serialization-length)
    ;; std_msgs/String _path
    (send _path :serialization-length)
    ;; std_msgs/Float64 _d1
    (send _d1 :serialization-length)
    ;; std_msgs/Float64 _d2
    (send _d2 :serialization-length)
    ;; std_msgs/Float64 _origin_dist
    (send _origin_dist :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Int32 _ID
       (send _ID :serialize s)
     ;; std_msgs/Float64 _x
       (send _x :serialize s)
     ;; std_msgs/Float64 _y
       (send _y :serialize s)
     ;; std_msgs/Float64 _vel
       (send _vel :serialize s)
     ;; std_msgs/String _path
       (send _path :serialize s)
     ;; std_msgs/Float64 _d1
       (send _d1 :serialize s)
     ;; std_msgs/Float64 _d2
       (send _d2 :serialize s)
     ;; std_msgs/Float64 _origin_dist
       (send _origin_dist :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Int32 _ID
     (send _ID :deserialize buf ptr-) (incf ptr- (send _ID :serialization-length))
   ;; std_msgs/Float64 _x
     (send _x :deserialize buf ptr-) (incf ptr- (send _x :serialization-length))
   ;; std_msgs/Float64 _y
     (send _y :deserialize buf ptr-) (incf ptr- (send _y :serialization-length))
   ;; std_msgs/Float64 _vel
     (send _vel :deserialize buf ptr-) (incf ptr- (send _vel :serialization-length))
   ;; std_msgs/String _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;; std_msgs/Float64 _d1
     (send _d1 :deserialize buf ptr-) (incf ptr- (send _d1 :serialization-length))
   ;; std_msgs/Float64 _d2
     (send _d2 :deserialize buf ptr-) (incf ptr- (send _d2 :serialization-length))
   ;; std_msgs/Float64 _origin_dist
     (send _origin_dist :deserialize buf ptr-) (incf ptr- (send _origin_dist :serialization-length))
   ;;
   self)
  )

(setf (get cav_project::limo_info :md5sum-) "120a9aa2d6a27cbacb07fac971279075")
(setf (get cav_project::limo_info :datatype-) "cav_project/limo_info")
(setf (get cav_project::limo_info :definition-)
      "std_msgs/Int32 ID
std_msgs/Float64 x
std_msgs/Float64 y
std_msgs/Float64 vel
std_msgs/String path
std_msgs/Float64 d1
std_msgs/Float64 d2
std_msgs/Float64 origin_dist

================================================================================
MSG: std_msgs/Int32
int32 data
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: std_msgs/String
string data

")



(provide :cav_project/limo_info "120a9aa2d6a27cbacb07fac971279075")


