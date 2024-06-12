;; Auto-generated. Do not edit!


(when (boundp 'cav_project::limo_info_array)
  (if (not (find-package "CAV_PROJECT"))
    (make-package "CAV_PROJECT"))
  (shadow 'limo_info_array (find-package "CAV_PROJECT")))
(unless (find-package "CAV_PROJECT::LIMO_INFO_ARRAY")
  (make-package "CAV_PROJECT::LIMO_INFO_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude limo_info_array.msg.html


(defclass cav_project::limo_info_array
  :super ros::object
  :slots (_limo_infos ))

(defmethod cav_project::limo_info_array
  (:init
   (&key
    ((:limo_infos __limo_infos) ())
    )
   (send-super :init)
   (setq _limo_infos __limo_infos)
   self)
  (:limo_infos
   (&rest __limo_infos)
   (if (keywordp (car __limo_infos))
       (send* _limo_infos __limo_infos)
     (progn
       (if __limo_infos (setq _limo_infos (car __limo_infos)))
       _limo_infos)))
  (:serialization-length
   ()
   (+
    ;; cav_project/limo_info[] _limo_infos
    (apply #'+ (send-all _limo_infos :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; cav_project/limo_info[] _limo_infos
     (write-long (length _limo_infos) s)
     (dolist (elem _limo_infos)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; cav_project/limo_info[] _limo_infos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _limo_infos (let (r) (dotimes (i n) (push (instance cav_project::limo_info :init) r)) r))
     (dolist (elem- _limo_infos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cav_project::limo_info_array :md5sum-) "5a41342e126c12696bddf612e49f157c")
(setf (get cav_project::limo_info_array :datatype-) "cav_project/limo_info_array")
(setf (get cav_project::limo_info_array :definition-)
      "cav_project/limo_info[] limo_infos

================================================================================
MSG: cav_project/limo_info
std_msgs/Float64 vel

================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :cav_project/limo_info_array "5a41342e126c12696bddf612e49f157c")


