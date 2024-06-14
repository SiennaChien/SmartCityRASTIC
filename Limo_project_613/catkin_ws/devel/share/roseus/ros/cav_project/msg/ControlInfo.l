;; Auto-generated. Do not edit!


(when (boundp 'cav_project::ControlInfo)
  (if (not (find-package "CAV_PROJECT"))
    (make-package "CAV_PROJECT"))
  (shadow 'ControlInfo (find-package "CAV_PROJECT")))
(unless (find-package "CAV_PROJECT::CONTROLINFO")
  (make-package "CAV_PROJECT::CONTROLINFO"))

(in-package "ROS")
;;//! \htmlinclude ControlInfo.msg.html


(defclass cav_project::ControlInfo
  :super ros::object
  :slots (_steering_angle _desired_velocity _control_input ))

(defmethod cav_project::ControlInfo
  (:init
   (&key
    ((:steering_angle __steering_angle) 0.0)
    ((:desired_velocity __desired_velocity) 0.0)
    ((:control_input __control_input) 0.0)
    )
   (send-super :init)
   (setq _steering_angle (float __steering_angle))
   (setq _desired_velocity (float __desired_velocity))
   (setq _control_input (float __control_input))
   self)
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:desired_velocity
   (&optional __desired_velocity)
   (if __desired_velocity (setq _desired_velocity __desired_velocity)) _desired_velocity)
  (:control_input
   (&optional __control_input)
   (if __control_input (setq _control_input __control_input)) _control_input)
  (:serialization-length
   ()
   (+
    ;; float64 _steering_angle
    8
    ;; float64 _desired_velocity
    8
    ;; float64 _control_input
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _desired_velocity
       (sys::poke _desired_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _control_input
       (sys::poke _control_input (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _desired_velocity
     (setq _desired_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _control_input
     (setq _control_input (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get cav_project::ControlInfo :md5sum-) "6a24bd3f8a8eca54ba890a9158619a2e")
(setf (get cav_project::ControlInfo :datatype-) "cav_project/ControlInfo")
(setf (get cav_project::ControlInfo :definition-)
      "float64 steering_angle
float64 desired_velocity
float64 control_input

")



(provide :cav_project/ControlInfo "6a24bd3f8a8eca54ba890a9158619a2e")


