;; Auto-generated. Do not edit!


(when (boundp 'cav_project::QP_solution)
  (if (not (find-package "CAV_PROJECT"))
    (make-package "CAV_PROJECT"))
  (shadow 'QP_solution (find-package "CAV_PROJECT")))
(unless (find-package "CAV_PROJECT::QP_SOLUTION")
  (make-package "CAV_PROJECT::QP_SOLUTION"))

(in-package "ROS")
;;//! \htmlinclude QP_solution.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cav_project::QP_solution
  :super ros::object
  :slots (_u ))

(defmethod cav_project::QP_solution
  (:init
   (&key
    ((:u __u) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _u __u)
   self)
  (:u
   (&rest __u)
   (if (keywordp (car __u))
       (send* _u __u)
     (progn
       (if __u (setq _u (car __u)))
       _u)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64 _u
    (send _u :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64 _u
       (send _u :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64 _u
     (send _u :deserialize buf ptr-) (incf ptr- (send _u :serialization-length))
   ;;
   self)
  )

(setf (get cav_project::QP_solution :md5sum-) "a9be02629cec3211ddab55e654369a74")
(setf (get cav_project::QP_solution :datatype-) "cav_project/QP_solution")
(setf (get cav_project::QP_solution :definition-)
      "std_msgs/Float64 u

================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :cav_project/QP_solution "a9be02629cec3211ddab55e654369a74")


