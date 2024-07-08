;; Auto-generated. Do not edit!


(when (boundp 'cav_project::QP_solution)
  (if (not (find-package "CAV_PROJECT"))
    (make-package "CAV_PROJECT"))
  (shadow 'QP_solution (find-package "CAV_PROJECT")))
(unless (find-package "CAV_PROJECT::QP_SOLUTION")
  (make-package "CAV_PROJECT::QP_SOLUTION"))

(in-package "ROS")
;;//! \htmlinclude QP_solution.msg.html


(defclass cav_project::QP_solution
  :super ros::object
  :slots (_u ))

(defmethod cav_project::QP_solution
  (:init
   (&key
    ((:u __u) 0.0)
    )
   (send-super :init)
   (setq _u (float __u))
   self)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:serialization-length
   ()
   (+
    ;; float64 _u
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _u
       (sys::poke _u (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _u
     (setq _u (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get cav_project::QP_solution :md5sum-) "988df341e727ad40b85d2b8acf9471e9")
(setf (get cav_project::QP_solution :datatype-) "cav_project/QP_solution")
(setf (get cav_project::QP_solution :definition-)
      "float64 u

")



(provide :cav_project/QP_solution "988df341e727ad40b85d2b8acf9471e9")


