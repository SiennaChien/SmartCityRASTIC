; Auto-generated. Do not edit!


(cl:in-package cav_project-msg)


;//! \htmlinclude limo_info.msg.html

(cl:defclass <limo_info> (roslisp-msg-protocol:ros-message)
  ((vel
    :reader vel
    :initarg :vel
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass limo_info (<limo_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <limo_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'limo_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cav_project-msg:<limo_info> is deprecated: use cav_project-msg:limo_info instead.")))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <limo_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:vel-val is deprecated.  Use cav_project-msg:vel instead.")
  (vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <limo_info>) ostream)
  "Serializes a message object of type '<limo_info>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <limo_info>) istream)
  "Deserializes a message object of type '<limo_info>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel) istream)
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
  "b2990a44cec4fde7af6f58f2727169f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'limo_info)))
  "Returns md5sum for a message object of type 'limo_info"
  "b2990a44cec4fde7af6f58f2727169f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<limo_info>)))
  "Returns full string definition for message of type '<limo_info>"
  (cl:format cl:nil "std_msgs/Float64 vel~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'limo_info)))
  "Returns full string definition for message of type 'limo_info"
  (cl:format cl:nil "std_msgs/Float64 vel~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <limo_info>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <limo_info>))
  "Converts a ROS message object to a list"
  (cl:list 'limo_info
    (cl:cons ':vel (vel msg))
))
