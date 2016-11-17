; Auto-generated. Do not edit!


(cl:in-package robotis_controller_msgs-msg)


;//! \htmlinclude ControlTorque.msg.html

(cl:defclass <ControlTorque> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (enable
    :reader enable
    :initarg :enable
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass ControlTorque (<ControlTorque>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlTorque>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlTorque)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-msg:<ControlTorque> is deprecated: use robotis_controller_msgs-msg:ControlTorque instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ControlTorque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:name-val is deprecated.  Use robotis_controller_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <ControlTorque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:enable-val is deprecated.  Use robotis_controller_msgs-msg:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlTorque>) ostream)
  "Serializes a message object of type '<ControlTorque>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'enable))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'enable))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlTorque>) istream)
  "Deserializes a message object of type '<ControlTorque>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'name) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'name)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'enable) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'enable)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlTorque>)))
  "Returns string type for a message object of type '<ControlTorque>"
  "robotis_controller_msgs/ControlTorque")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlTorque)))
  "Returns string type for a message object of type 'ControlTorque"
  "robotis_controller_msgs/ControlTorque")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlTorque>)))
  "Returns md5sum for a message object of type '<ControlTorque>"
  "c6b11d561803e96cfa68e7be9c607e6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlTorque)))
  "Returns md5sum for a message object of type 'ControlTorque"
  "c6b11d561803e96cfa68e7be9c607e6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlTorque>)))
  "Returns full string definition for message of type '<ControlTorque>"
  (cl:format cl:nil "string[] name~%bool[] enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlTorque)))
  "Returns full string definition for message of type 'ControlTorque"
  (cl:format cl:nil "string[] name~%bool[] enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlTorque>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'enable) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlTorque>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlTorque
    (cl:cons ':name (name msg))
    (cl:cons ':enable (enable msg))
))
