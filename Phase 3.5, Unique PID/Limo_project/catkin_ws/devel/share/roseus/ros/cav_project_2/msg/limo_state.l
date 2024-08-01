;; Auto-generated. Do not edit!


(when (boundp 'cav_project_2::limo_state)
  (if (not (find-package "CAV_PROJECT_2"))
    (make-package "CAV_PROJECT_2"))
  (shadow 'limo_state (find-package "CAV_PROJECT_2")))
(unless (find-package "CAV_PROJECT_2::LIMO_STATE")
  (make-package "CAV_PROJECT_2::LIMO_STATE"))

(in-package "ROS")
;;//! \htmlinclude limo_state.msg.html


(defclass cav_project_2::limo_state
  :super ros::object
  :slots (_limoID _vel _d0 _d1 _v1 _d2 _v2 _vd ))

(defmethod cav_project_2::limo_state
  (:init
   (&key
    ((:limoID __limoID) "")
    ((:vel __vel) 0.0)
    ((:d0 __d0) 0.0)
    ((:d1 __d1) 0.0)
    ((:v1 __v1) 0.0)
    ((:d2 __d2) 0.0)
    ((:v2 __v2) 0.0)
    ((:vd __vd) 0.0)
    )
   (send-super :init)
   (setq _limoID (string __limoID))
   (setq _vel (float __vel))
   (setq _d0 (float __d0))
   (setq _d1 (float __d1))
   (setq _v1 (float __v1))
   (setq _d2 (float __d2))
   (setq _v2 (float __v2))
   (setq _vd (float __vd))
   self)
  (:limoID
   (&optional __limoID)
   (if __limoID (setq _limoID __limoID)) _limoID)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:d0
   (&optional __d0)
   (if __d0 (setq _d0 __d0)) _d0)
  (:d1
   (&optional __d1)
   (if __d1 (setq _d1 __d1)) _d1)
  (:v1
   (&optional __v1)
   (if __v1 (setq _v1 __v1)) _v1)
  (:d2
   (&optional __d2)
   (if __d2 (setq _d2 __d2)) _d2)
  (:v2
   (&optional __v2)
   (if __v2 (setq _v2 __v2)) _v2)
  (:vd
   (&optional __vd)
   (if __vd (setq _vd __vd)) _vd)
  (:serialization-length
   ()
   (+
    ;; string _limoID
    4 (length _limoID)
    ;; float64 _vel
    8
    ;; float64 _d0
    8
    ;; float64 _d1
    8
    ;; float64 _v1
    8
    ;; float64 _d2
    8
    ;; float64 _v2
    8
    ;; float64 _vd
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _limoID
       (write-long (length _limoID) s) (princ _limoID s)
     ;; float64 _vel
       (sys::poke _vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d0
       (sys::poke _d0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d1
       (sys::poke _d1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v1
       (sys::poke _v1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d2
       (sys::poke _d2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v2
       (sys::poke _v2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vd
       (sys::poke _vd (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _limoID
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _limoID (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _vel
     (setq _vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d0
     (setq _d0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d1
     (setq _d1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v1
     (setq _v1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d2
     (setq _d2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v2
     (setq _v2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vd
     (setq _vd (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get cav_project_2::limo_state :md5sum-) "a1fba4736fa7b1723499e2717d3f0429")
(setf (get cav_project_2::limo_state :datatype-) "cav_project_2/limo_state")
(setf (get cav_project_2::limo_state :definition-)
      "string limoID
float64 vel
float64 d0
float64 d1
float64 v1
float64 d2
float64 v2
float64 vd

")



(provide :cav_project_2/limo_state "a1fba4736fa7b1723499e2717d3f0429")


