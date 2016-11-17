; Auto-generated. Do not edit!


(cl:in-package cat_move_to_target-srv)


;//! \htmlinclude GetJointValues-request.msg.html

(cl:defclass <GetJointValues-request> (roslisp-msg-protocol:ros-message)
  ((pos_x
    :reader pos_x
    :initarg :pos_x
    :type cl:float
    :initform 0.0)
   (pos_y
    :reader pos_y
    :initarg :pos_y
    :type cl:float
    :initform 0.0)
   (pos_z
    :reader pos_z
    :initarg :pos_z
    :type cl:float
    :initform 0.0)
   (zyx_angle_z
    :reader zyx_angle_z
    :initarg :zyx_angle_z
    :type cl:float
    :initform 0.0)
   (zyx_angle_y
    :reader zyx_angle_y
    :initarg :zyx_angle_y
    :type cl:float
    :initform 0.0)
   (zyx_angle_x
    :reader zyx_angle_x
    :initarg :zyx_angle_x
    :type cl:float
    :initform 0.0)
   (hardcoded_wrist
    :reader hardcoded_wrist
    :initarg :hardcoded_wrist
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetJointValues-request (<GetJointValues-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointValues-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointValues-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cat_move_to_target-srv:<GetJointValues-request> is deprecated: use cat_move_to_target-srv:GetJointValues-request instead.")))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <GetJointValues-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:pos_x-val is deprecated.  Use cat_move_to_target-srv:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <GetJointValues-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:pos_y-val is deprecated.  Use cat_move_to_target-srv:pos_y instead.")
  (pos_y m))

(cl:ensure-generic-function 'pos_z-val :lambda-list '(m))
(cl:defmethod pos_z-val ((m <GetJointValues-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:pos_z-val is deprecated.  Use cat_move_to_target-srv:pos_z instead.")
  (pos_z m))

(cl:ensure-generic-function 'zyx_angle_z-val :lambda-list '(m))
(cl:defmethod zyx_angle_z-val ((m <GetJointValues-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:zyx_angle_z-val is deprecated.  Use cat_move_to_target-srv:zyx_angle_z instead.")
  (zyx_angle_z m))

(cl:ensure-generic-function 'zyx_angle_y-val :lambda-list '(m))
(cl:defmethod zyx_angle_y-val ((m <GetJointValues-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:zyx_angle_y-val is deprecated.  Use cat_move_to_target-srv:zyx_angle_y instead.")
  (zyx_angle_y m))

(cl:ensure-generic-function 'zyx_angle_x-val :lambda-list '(m))
(cl:defmethod zyx_angle_x-val ((m <GetJointValues-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:zyx_angle_x-val is deprecated.  Use cat_move_to_target-srv:zyx_angle_x instead.")
  (zyx_angle_x m))

(cl:ensure-generic-function 'hardcoded_wrist-val :lambda-list '(m))
(cl:defmethod hardcoded_wrist-val ((m <GetJointValues-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:hardcoded_wrist-val is deprecated.  Use cat_move_to_target-srv:hardcoded_wrist instead.")
  (hardcoded_wrist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointValues-request>) ostream)
  "Serializes a message object of type '<GetJointValues-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'zyx_angle_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'zyx_angle_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'zyx_angle_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hardcoded_wrist) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointValues-request>) istream)
  "Deserializes a message object of type '<GetJointValues-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zyx_angle_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zyx_angle_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zyx_angle_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'hardcoded_wrist) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointValues-request>)))
  "Returns string type for a service object of type '<GetJointValues-request>"
  "cat_move_to_target/GetJointValuesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointValues-request)))
  "Returns string type for a service object of type 'GetJointValues-request"
  "cat_move_to_target/GetJointValuesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointValues-request>)))
  "Returns md5sum for a message object of type '<GetJointValues-request>"
  "7a5da121fb59cc8b0d5a404807350ade")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointValues-request)))
  "Returns md5sum for a message object of type 'GetJointValues-request"
  "7a5da121fb59cc8b0d5a404807350ade")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointValues-request>)))
  "Returns full string definition for message of type '<GetJointValues-request>"
  (cl:format cl:nil "float64 pos_x~%float64 pos_y~%float64 pos_z~%float64 zyx_angle_z~%float64 zyx_angle_y~%float64 zyx_angle_x~%bool hardcoded_wrist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointValues-request)))
  "Returns full string definition for message of type 'GetJointValues-request"
  (cl:format cl:nil "float64 pos_x~%float64 pos_y~%float64 pos_z~%float64 zyx_angle_z~%float64 zyx_angle_y~%float64 zyx_angle_x~%bool hardcoded_wrist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointValues-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointValues-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointValues-request
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
    (cl:cons ':pos_z (pos_z msg))
    (cl:cons ':zyx_angle_z (zyx_angle_z msg))
    (cl:cons ':zyx_angle_y (zyx_angle_y msg))
    (cl:cons ':zyx_angle_x (zyx_angle_x msg))
    (cl:cons ':hardcoded_wrist (hardcoded_wrist msg))
))
;//! \htmlinclude GetJointValues-response.msg.html

(cl:defclass <GetJointValues-response> (roslisp-msg-protocol:ros-message)
  ((theta1
    :reader theta1
    :initarg :theta1
    :type cl:float
    :initform 0.0)
   (theta2
    :reader theta2
    :initarg :theta2
    :type cl:float
    :initform 0.0)
   (theta3
    :reader theta3
    :initarg :theta3
    :type cl:float
    :initform 0.0)
   (theta4
    :reader theta4
    :initarg :theta4
    :type cl:float
    :initform 0.0)
   (theta5
    :reader theta5
    :initarg :theta5
    :type cl:float
    :initform 0.0)
   (theta6
    :reader theta6
    :initarg :theta6
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetJointValues-response (<GetJointValues-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointValues-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointValues-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cat_move_to_target-srv:<GetJointValues-response> is deprecated: use cat_move_to_target-srv:GetJointValues-response instead.")))

(cl:ensure-generic-function 'theta1-val :lambda-list '(m))
(cl:defmethod theta1-val ((m <GetJointValues-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:theta1-val is deprecated.  Use cat_move_to_target-srv:theta1 instead.")
  (theta1 m))

(cl:ensure-generic-function 'theta2-val :lambda-list '(m))
(cl:defmethod theta2-val ((m <GetJointValues-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:theta2-val is deprecated.  Use cat_move_to_target-srv:theta2 instead.")
  (theta2 m))

(cl:ensure-generic-function 'theta3-val :lambda-list '(m))
(cl:defmethod theta3-val ((m <GetJointValues-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:theta3-val is deprecated.  Use cat_move_to_target-srv:theta3 instead.")
  (theta3 m))

(cl:ensure-generic-function 'theta4-val :lambda-list '(m))
(cl:defmethod theta4-val ((m <GetJointValues-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:theta4-val is deprecated.  Use cat_move_to_target-srv:theta4 instead.")
  (theta4 m))

(cl:ensure-generic-function 'theta5-val :lambda-list '(m))
(cl:defmethod theta5-val ((m <GetJointValues-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:theta5-val is deprecated.  Use cat_move_to_target-srv:theta5 instead.")
  (theta5 m))

(cl:ensure-generic-function 'theta6-val :lambda-list '(m))
(cl:defmethod theta6-val ((m <GetJointValues-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:theta6-val is deprecated.  Use cat_move_to_target-srv:theta6 instead.")
  (theta6 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointValues-response>) ostream)
  "Serializes a message object of type '<GetJointValues-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta6))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointValues-response>) istream)
  "Deserializes a message object of type '<GetJointValues-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta4) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta5) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta6) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointValues-response>)))
  "Returns string type for a service object of type '<GetJointValues-response>"
  "cat_move_to_target/GetJointValuesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointValues-response)))
  "Returns string type for a service object of type 'GetJointValues-response"
  "cat_move_to_target/GetJointValuesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointValues-response>)))
  "Returns md5sum for a message object of type '<GetJointValues-response>"
  "7a5da121fb59cc8b0d5a404807350ade")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointValues-response)))
  "Returns md5sum for a message object of type 'GetJointValues-response"
  "7a5da121fb59cc8b0d5a404807350ade")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointValues-response>)))
  "Returns full string definition for message of type '<GetJointValues-response>"
  (cl:format cl:nil "float64 theta1~%float64 theta2~%float64 theta3~%float64 theta4~%float64 theta5~%float64 theta6~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointValues-response)))
  "Returns full string definition for message of type 'GetJointValues-response"
  (cl:format cl:nil "float64 theta1~%float64 theta2~%float64 theta3~%float64 theta4~%float64 theta5~%float64 theta6~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointValues-response>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointValues-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointValues-response
    (cl:cons ':theta1 (theta1 msg))
    (cl:cons ':theta2 (theta2 msg))
    (cl:cons ':theta3 (theta3 msg))
    (cl:cons ':theta4 (theta4 msg))
    (cl:cons ':theta5 (theta5 msg))
    (cl:cons ':theta6 (theta6 msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetJointValues)))
  'GetJointValues-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetJointValues)))
  'GetJointValues-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointValues)))
  "Returns string type for a service object of type '<GetJointValues>"
  "cat_move_to_target/GetJointValues")