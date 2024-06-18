; Auto-generated. Do not edit!


(cl:in-package cav_project-msg)


;//! \htmlinclude limo_state_matrix.msg.html

(cl:defclass <limo_state_matrix> (roslisp-msg-protocol:ros-message)
  ((limos
    :reader limos
    :initarg :limos
    :type (cl:vector cav_project-msg:limo_state)
   :initform (cl:make-array 0 :element-type 'cav_project-msg:limo_state :initial-element (cl:make-instance 'cav_project-msg:limo_state))))
)

(cl:defclass limo_state_matrix (<limo_state_matrix>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <limo_state_matrix>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'limo_state_matrix)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cav_project-msg:<limo_state_matrix> is deprecated: use cav_project-msg:limo_state_matrix instead.")))

(cl:ensure-generic-function 'limos-val :lambda-list '(m))
(cl:defmethod limos-val ((m <limo_state_matrix>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:limos-val is deprecated.  Use cav_project-msg:limos instead.")
  (limos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <limo_state_matrix>) ostream)
  "Serializes a message object of type '<limo_state_matrix>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'limos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'limos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <limo_state_matrix>) istream)
  "Deserializes a message object of type '<limo_state_matrix>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'limos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'limos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cav_project-msg:limo_state))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<limo_state_matrix>)))
  "Returns string type for a message object of type '<limo_state_matrix>"
  "cav_project/limo_state_matrix")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'limo_state_matrix)))
  "Returns string type for a message object of type 'limo_state_matrix"
  "cav_project/limo_state_matrix")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<limo_state_matrix>)))
  "Returns md5sum for a message object of type '<limo_state_matrix>"
  "8726c45313017f9857b70bc15ddc333f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'limo_state_matrix)))
  "Returns md5sum for a message object of type 'limo_state_matrix"
  "8726c45313017f9857b70bc15ddc333f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<limo_state_matrix>)))
  "Returns full string definition for message of type '<limo_state_matrix>"
  (cl:format cl:nil "limo_state[] limos~%~%================================================================================~%MSG: cav_project/limo_state~%string limoID~%float64 vel~%float64 d0~%float64 d1~%float64 v1~%float64 d2~%float64 v2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'limo_state_matrix)))
  "Returns full string definition for message of type 'limo_state_matrix"
  (cl:format cl:nil "limo_state[] limos~%~%================================================================================~%MSG: cav_project/limo_state~%string limoID~%float64 vel~%float64 d0~%float64 d1~%float64 v1~%float64 d2~%float64 v2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <limo_state_matrix>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'limos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <limo_state_matrix>))
  "Converts a ROS message object to a list"
  (cl:list 'limo_state_matrix
    (cl:cons ':limos (limos msg))
))
