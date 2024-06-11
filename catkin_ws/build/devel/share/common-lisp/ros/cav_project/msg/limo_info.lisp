; Auto-generated. Do not edit!


(cl:in-package cav_project-msg)


;//! \htmlinclude limo_info.msg.html

(cl:defclass <limo_info> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (x
    :reader x
    :initarg :x
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (y
    :reader y
    :initarg :y
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (vel
    :reader vel
    :initarg :vel
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (path
    :reader path
    :initarg :path
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (d1
    :reader d1
    :initarg :d1
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (d2
    :reader d2
    :initarg :d2
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (origin_dist
    :reader origin_dist
    :initarg :origin_dist
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass limo_info (<limo_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <limo_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'limo_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cav_project-msg:<limo_info> is deprecated: use cav_project-msg:limo_info instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <limo_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:ID-val is deprecated.  Use cav_project-msg:ID instead.")
  (ID m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <limo_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:x-val is deprecated.  Use cav_project-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <limo_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:y-val is deprecated.  Use cav_project-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <limo_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:vel-val is deprecated.  Use cav_project-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <limo_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:path-val is deprecated.  Use cav_project-msg:path instead.")
  (path m))

(cl:ensure-generic-function 'd1-val :lambda-list '(m))
(cl:defmethod d1-val ((m <limo_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:d1-val is deprecated.  Use cav_project-msg:d1 instead.")
  (d1 m))

(cl:ensure-generic-function 'd2-val :lambda-list '(m))
(cl:defmethod d2-val ((m <limo_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:d2-val is deprecated.  Use cav_project-msg:d2 instead.")
  (d2 m))

(cl:ensure-generic-function 'origin_dist-val :lambda-list '(m))
(cl:defmethod origin_dist-val ((m <limo_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:origin_dist-val is deprecated.  Use cav_project-msg:origin_dist instead.")
  (origin_dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <limo_info>) ostream)
  "Serializes a message object of type '<limo_info>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ID) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'x) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'y) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'd1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'd2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin_dist) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <limo_info>) istream)
  "Deserializes a message object of type '<limo_info>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ID) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'x) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'y) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'd1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'd2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin_dist) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<limo_info>)))
  "Returns string type for a message object of type '<limo_info>"
  "cav_project/limo_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'limo_info)))
  "Returns string type for a message object of type 'limo_info"
  "cav_project/limo_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<limo_info>)))
  "Returns md5sum for a message object of type '<limo_info>"
  "120a9aa2d6a27cbacb07fac971279075")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'limo_info)))
  "Returns md5sum for a message object of type 'limo_info"
  "120a9aa2d6a27cbacb07fac971279075")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<limo_info>)))
  "Returns full string definition for message of type '<limo_info>"
  (cl:format cl:nil "std_msgs/Int32 ID~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/Float64 vel~%std_msgs/String path~%std_msgs/Float64 d1~%std_msgs/Float64 d2~%std_msgs/Float64 origin_dist~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'limo_info)))
  "Returns full string definition for message of type 'limo_info"
  (cl:format cl:nil "std_msgs/Int32 ID~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/Float64 vel~%std_msgs/String path~%std_msgs/Float64 d1~%std_msgs/Float64 d2~%std_msgs/Float64 origin_dist~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <limo_info>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ID))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'x))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'y))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'd1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'd2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin_dist))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <limo_info>))
  "Converts a ROS message object to a list"
  (cl:list 'limo_info
    (cl:cons ':ID (ID msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':path (path msg))
    (cl:cons ':d1 (d1 msg))
    (cl:cons ':d2 (d2 msg))
    (cl:cons ':origin_dist (origin_dist msg))
))
