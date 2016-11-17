; Auto-generated. Do not edit!


(cl:in-package robotis_controller_msgs-msg)


;//! \htmlinclude ControlWrite.msg.html

(cl:defclass <ControlWrite> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (addr
    :reader addr
    :initarg :addr
    :type cl:fixnum
    :initform 0)
   (length
    :reader length
    :initarg :length
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass ControlWrite (<ControlWrite>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlWrite>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlWrite)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotis_controller_msgs-msg:<ControlWrite> is deprecated: use robotis_controller_msgs-msg:ControlWrite instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ControlWrite>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:name-val is deprecated.  Use robotis_controller_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'addr-val :lambda-list '(m))
(cl:defmethod addr-val ((m <ControlWrite>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:addr-val is deprecated.  Use robotis_controller_msgs-msg:addr instead.")
  (addr m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <ControlWrite>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:length-val is deprecated.  Use robotis_controller_msgs-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ControlWrite>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotis_controller_msgs-msg:value-val is deprecated.  Use robotis_controller_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlWrite>) ostream)
  "Serializes a message object of type '<ControlWrite>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'addr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlWrite>) istream)
  "Deserializes a message object of type '<ControlWrite>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'addr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'value)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlWrite>)))
  "Returns string type for a message object of type '<ControlWrite>"
  "robotis_controller_msgs/ControlWrite")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlWrite)))
  "Returns string type for a message object of type 'ControlWrite"
  "robotis_controller_msgs/ControlWrite")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlWrite>)))
  "Returns md5sum for a message object of type '<ControlWrite>"
  "f56d9435515b596a4742dbb69832f87e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlWrite)))
  "Returns md5sum for a message object of type 'ControlWrite"
  "f56d9435515b596a4742dbb69832f87e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlWrite>)))
  "Returns full string definition for message of type '<ControlWrite>"
  (cl:format cl:nil "string name~%uint16 addr~%uint16 length~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlWrite)))
  "Returns full string definition for message of type 'ControlWrite"
  (cl:format cl:nil "string name~%uint16 addr~%uint16 length~%uint32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlWrite>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlWrite>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlWrite
    (cl:cons ':name (name msg))
    (cl:cons ':addr (addr msg))
    (cl:cons ':length (length msg))
    (cl:cons ':value (value msg))
))
