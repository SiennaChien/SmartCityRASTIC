; Auto-generated. Do not edit!


(cl:in-package cav_project-msg)


;//! \htmlinclude QP_solution.msg.html

(cl:defclass <QP_solution> (roslisp-msg-protocol:ros-message)
  ((u
    :reader u
    :initarg :u
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass QP_solution (<QP_solution>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QP_solution>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QP_solution)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cav_project-msg:<QP_solution> is deprecated: use cav_project-msg:QP_solution instead.")))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <QP_solution>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:u-val is deprecated.  Use cav_project-msg:u instead.")
  (u m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QP_solution>) ostream)
  "Serializes a message object of type '<QP_solution>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'u) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QP_solution>) istream)
  "Deserializes a message object of type '<QP_solution>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'u) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QP_solution>)))
  "Returns string type for a message object of type '<QP_solution>"
  "cav_project/QP_solution")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QP_solution)))
  "Returns string type for a message object of type 'QP_solution"
  "cav_project/QP_solution")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QP_solution>)))
  "Returns md5sum for a message object of type '<QP_solution>"
  "a9be02629cec3211ddab55e654369a74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QP_solution)))
  "Returns md5sum for a message object of type 'QP_solution"
  "a9be02629cec3211ddab55e654369a74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QP_solution>)))
  "Returns full string definition for message of type '<QP_solution>"
  (cl:format cl:nil "std_msgs/Float64 u~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QP_solution)))
  "Returns full string definition for message of type 'QP_solution"
  (cl:format cl:nil "std_msgs/Float64 u~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QP_solution>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'u))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QP_solution>))
  "Converts a ROS message object to a list"
  (cl:list 'QP_solution
    (cl:cons ':u (u msg))
))
