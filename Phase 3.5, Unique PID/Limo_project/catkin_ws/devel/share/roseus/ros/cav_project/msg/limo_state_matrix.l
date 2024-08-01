;; Auto-generated. Do not edit!


(when (boundp 'cav_project::limo_state_matrix)
  (if (not (find-package "CAV_PROJECT"))
    (make-package "CAV_PROJECT"))
  (shadow 'limo_state_matrix (find-package "CAV_PROJECT")))
(unless (find-package "CAV_PROJECT::LIMO_STATE_MATRIX")
  (make-package "CAV_PROJECT::LIMO_STATE_MATRIX"))

(in-package "ROS")
;;//! \htmlinclude limo_state_matrix.msg.html


(defclass cav_project::limo_state_matrix
  :super ros::object
  :slots (_limos ))

(defmethod cav_project::limo_state_matrix
  (:init
   (&key
    ((:limos __limos) ())
    )
   (send-super :init)
   (setq _limos __limos)
   self)
  (:limos
   (&rest __limos)
   (if (keywordp (car __limos))
       (send* _limos __limos)
     (progn
       (if __limos (setq _limos (car __limos)))
       _limos)))
  (:serialization-length
   ()
   (+
    ;; cav_project/limo_state[] _limos
    (apply #'+ (send-all _limos :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; cav_project/limo_state[] _limos
     (write-long (length _limos) s)
     (dolist (elem _limos)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; cav_project/limo_state[] _limos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _limos (let (r) (dotimes (i n) (push (instance cav_project::limo_state :init) r)) r))
     (dolist (elem- _limos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cav_project::limo_state_matrix :md5sum-) "16904eb3a77a18c79afd9f6ff63a6e61")
(setf (get cav_project::limo_state_matrix :datatype-) "cav_project/limo_state_matrix")
(setf (get cav_project::limo_state_matrix :definition-)
      "limo_state[] limos

================================================================================
MSG: cav_project/limo_state
string limoID
float64 vel
float64 d0
float64 d1
float64 v1
float64 d2
float64 v2
float64 l2
float64 d3
float64 v3
float64 l3
float64 vd

")



(provide :cav_project/limo_state_matrix "16904eb3a77a18c79afd9f6ff63a6e61")


