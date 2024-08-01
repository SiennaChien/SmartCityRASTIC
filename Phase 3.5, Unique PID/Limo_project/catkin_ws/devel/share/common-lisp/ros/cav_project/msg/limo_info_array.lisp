; Auto-generated. Do not edit!


(cl:in-package cav_project-msg)


;//! \htmlinclude limo_info_array.msg.html

(cl:defclass <limo_info_array> (roslisp-msg-protocol:ros-message)
  ((limo_infos
    :reader limo_infos
    :initarg :limo_infos
    :type (cl:vector cav_project-msg:limo_info)
   :initform (cl:make-array 0 :element-type 'cav_project-msg:limo_info :initial-element (cl:make-instance 'cav_project-msg:limo_info))))
)

(cl:defclass limo_info_array (<limo_info_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <limo_info_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'limo_info_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cav_project-msg:<limo_info_array> is deprecated: use cav_project-msg:limo_info_array instead.")))

(cl:ensure-generic-function 'limo_infos-val :lambda-list '(m))
(cl:defmethod limo_infos-val ((m <limo_info_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_project-msg:limo_infos-val is deprecated.  Use cav_project-msg:limo_infos instead.")
  (limo_infos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <limo_info_array>) ostream)
  "Serializes a message object of type '<limo_info_array>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'limo_infos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'limo_infos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <limo_info_array>) istream)
  "Deserializes a message object of type '<limo_info_array>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'limo_infos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'limo_infos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cav_project-msg:limo_info))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<limo_info_array>)))
  "Returns string type for a message object of type '<limo_info_array>"
  "cav_project/limo_info_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'limo_info_array)))
  "Returns string type for a message object of type 'limo_info_array"
  "cav_project/limo_info_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<limo_info_array>)))
  "Returns md5sum for a message object of type '<limo_info_array>"
  "5a41342e126c12696bddf612e49f157c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'limo_info_array)))
  "Returns md5sum for a message object of type 'limo_info_array"
  "5a41342e126c12696bddf612e49f157c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<limo_info_array>)))
  "Returns full string definition for message of type '<limo_info_array>"
  (cl:format cl:nil "cav_project/limo_info[] limo_infos~%~%================================================================================~%MSG: cav_project/limo_info~%std_msgs/Float64 vel~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'limo_info_array)))
  "Returns full string definition for message of type 'limo_info_array"
  (cl:format cl:nil "cav_project/limo_info[] limo_infos~%~%================================================================================~%MSG: cav_project/limo_info~%std_msgs/Float64 vel~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <limo_info_array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'limo_infos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <limo_info_array>))
  "Converts a ROS message object to a list"
  (cl:list 'limo_info_array
    (cl:cons ':limo_infos (limo_infos msg))
))
