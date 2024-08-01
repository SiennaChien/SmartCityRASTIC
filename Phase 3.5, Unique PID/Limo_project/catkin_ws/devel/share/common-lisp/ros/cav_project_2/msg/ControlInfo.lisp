; Auto-generated. Do not edit!


(cl:in-package cav_project_2-msg)


;//! \htmlinclude ControlInfo.msg.html

(cl:defclass <ControlInfo> (roslisp-msg-protocol:ros-message)
  ((steering_angle
    :reader steering_angle
    :initarg :steering_angle
    :type cl:float
    :initform 0.0)
   (desired_velocity
    :reader desired_velocity
    :initarg :desired_velocity
    :type cl:float
    :initform 0.0)
   (control_input
    :reader control_input
    :initarg :control_input
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlInfo (<ControlInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cav_project_2-msg:<ControlInfo> is deprecated: use cav_project_2-msg:ControlInfo instead.")))

(cl:ensure-generic-function 'steering_angle-val :lambda-list '(m))
(cl:defmethod steering_angle-val ((m <ControlInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project_2-msg:steering_angle-val is deprecated.  Use cav_project_2-msg:steering_angle instead.")
  (steering_angle m))

(cl:ensure-generic-function 'desired_velocity-val :lambda-list '(m))
(cl:defmethod desired_velocity-val ((m <ControlInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project_2-msg:desired_velocity-val is deprecated.  Use cav_project_2-msg:desired_velocity instead.")
  (desired_velocity m))

(cl:ensure-generic-function 'control_input-val :lambda-list '(m))
(cl:defmethod control_input-val ((m <ControlInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project_2-msg:control_input-val is deprecated.  Use cav_project_2-msg:control_input instead.")
  (control_input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlInfo>) ostream)
  "Serializes a message object of type '<ControlInfo>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'desired_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'control_input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlInfo>) istream)
  "Deserializes a message object of type '<ControlInfo>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desired_velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'control_input) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlInfo>)))
  "Returns string type for a message object of type '<ControlInfo>"
  "cav_project_2/ControlInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlInfo)))
  "Returns string type for a message object of type 'ControlInfo"
  "cav_project_2/ControlInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlInfo>)))
  "Returns md5sum for a message object of type '<ControlInfo>"
  "6a24bd3f8a8eca54ba890a9158619a2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlInfo)))
  "Returns md5sum for a message object of type 'ControlInfo"
  "6a24bd3f8a8eca54ba890a9158619a2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlInfo>)))
  "Returns full string definition for message of type '<ControlInfo>"
  (cl:format cl:nil "float64 steering_angle~%float64 desired_velocity~%float64 control_input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlInfo)))
  "Returns full string definition for message of type 'ControlInfo"
  (cl:format cl:nil "float64 steering_angle~%float64 desired_velocity~%float64 control_input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlInfo>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlInfo
    (cl:cons ':steering_angle (steering_angle msg))
    (cl:cons ':desired_velocity (desired_velocity msg))
    (cl:cons ':control_input (control_input msg))
))
