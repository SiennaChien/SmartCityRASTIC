; Auto-generated. Do not edit!


(cl:in-package cav_project-msg)


;//! \htmlinclude QP_solution.msg.html

(cl:defclass <QP_solution> (roslisp-msg-protocol:ros-message)
  ((u
    :reader u
    :initarg :u
    :type cl:float
    :initform 0.0))
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'u))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QP_solution>) istream)
  "Deserializes a message object of type '<QP_solution>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u) (roslisp-utils:decode-double-float-bits bits)))
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
  "988df341e727ad40b85d2b8acf9471e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QP_solution)))
  "Returns md5sum for a message object of type 'QP_solution"
  "988df341e727ad40b85d2b8acf9471e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QP_solution>)))
  "Returns full string definition for message of type '<QP_solution>"
  (cl:format cl:nil "float64 u~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QP_solution)))
  "Returns full string definition for message of type 'QP_solution"
  (cl:format cl:nil "float64 u~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QP_solution>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QP_solution>))
  "Converts a ROS message object to a list"
  (cl:list 'QP_solution
    (cl:cons ':u (u msg))
))
