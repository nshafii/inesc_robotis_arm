; Auto-generated. Do not edit!


(cl:in-package cat_move_to_target-srv)


;//! \htmlinclude GetTagPose-request.msg.html

(cl:defclass <GetTagPose-request> (roslisp-msg-protocol:ros-message)
  ((tag_id
    :reader tag_id
    :initarg :tag_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetTagPose-request (<GetTagPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTagPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTagPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cat_move_to_target-srv:<GetTagPose-request> is deprecated: use cat_move_to_target-srv:GetTagPose-request instead.")))

(cl:ensure-generic-function 'tag_id-val :lambda-list '(m))
(cl:defmethod tag_id-val ((m <GetTagPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:tag_id-val is deprecated.  Use cat_move_to_target-srv:tag_id instead.")
  (tag_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTagPose-request>) ostream)
  "Serializes a message object of type '<GetTagPose-request>"
  (cl:let* ((signed (cl:slot-value msg 'tag_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTagPose-request>) istream)
  "Deserializes a message object of type '<GetTagPose-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTagPose-request>)))
  "Returns string type for a service object of type '<GetTagPose-request>"
  "cat_move_to_target/GetTagPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTagPose-request)))
  "Returns string type for a service object of type 'GetTagPose-request"
  "cat_move_to_target/GetTagPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTagPose-request>)))
  "Returns md5sum for a message object of type '<GetTagPose-request>"
  "9947cf06bd814875243bead8e5c2eb9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTagPose-request)))
  "Returns md5sum for a message object of type 'GetTagPose-request"
  "9947cf06bd814875243bead8e5c2eb9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTagPose-request>)))
  "Returns full string definition for message of type '<GetTagPose-request>"
  (cl:format cl:nil "int16 tag_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTagPose-request)))
  "Returns full string definition for message of type 'GetTagPose-request"
  (cl:format cl:nil "int16 tag_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTagPose-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTagPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTagPose-request
    (cl:cons ':tag_id (tag_id msg))
))
;//! \htmlinclude GetTagPose-response.msg.html

(cl:defclass <GetTagPose-response> (roslisp-msg-protocol:ros-message)
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
   (ori_x
    :reader ori_x
    :initarg :ori_x
    :type cl:float
    :initform 0.0)
   (ori_y
    :reader ori_y
    :initarg :ori_y
    :type cl:float
    :initform 0.0)
   (ori_z
    :reader ori_z
    :initarg :ori_z
    :type cl:float
    :initform 0.0)
   (ori_w
    :reader ori_w
    :initarg :ori_w
    :type cl:float
    :initform 0.0)
   (w_pos_x
    :reader w_pos_x
    :initarg :w_pos_x
    :type cl:float
    :initform 0.0)
   (w_pos_y
    :reader w_pos_y
    :initarg :w_pos_y
    :type cl:float
    :initform 0.0)
   (w_pos_z
    :reader w_pos_z
    :initarg :w_pos_z
    :type cl:float
    :initform 0.0)
   (foundtag
    :reader foundtag
    :initarg :foundtag
    :type cl:boolean
    :initform cl:nil)
   (tag_id
    :reader tag_id
    :initarg :tag_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetTagPose-response (<GetTagPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTagPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTagPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cat_move_to_target-srv:<GetTagPose-response> is deprecated: use cat_move_to_target-srv:GetTagPose-response instead.")))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:pos_x-val is deprecated.  Use cat_move_to_target-srv:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:pos_y-val is deprecated.  Use cat_move_to_target-srv:pos_y instead.")
  (pos_y m))

(cl:ensure-generic-function 'pos_z-val :lambda-list '(m))
(cl:defmethod pos_z-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:pos_z-val is deprecated.  Use cat_move_to_target-srv:pos_z instead.")
  (pos_z m))

(cl:ensure-generic-function 'ori_x-val :lambda-list '(m))
(cl:defmethod ori_x-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:ori_x-val is deprecated.  Use cat_move_to_target-srv:ori_x instead.")
  (ori_x m))

(cl:ensure-generic-function 'ori_y-val :lambda-list '(m))
(cl:defmethod ori_y-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:ori_y-val is deprecated.  Use cat_move_to_target-srv:ori_y instead.")
  (ori_y m))

(cl:ensure-generic-function 'ori_z-val :lambda-list '(m))
(cl:defmethod ori_z-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:ori_z-val is deprecated.  Use cat_move_to_target-srv:ori_z instead.")
  (ori_z m))

(cl:ensure-generic-function 'ori_w-val :lambda-list '(m))
(cl:defmethod ori_w-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:ori_w-val is deprecated.  Use cat_move_to_target-srv:ori_w instead.")
  (ori_w m))

(cl:ensure-generic-function 'w_pos_x-val :lambda-list '(m))
(cl:defmethod w_pos_x-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:w_pos_x-val is deprecated.  Use cat_move_to_target-srv:w_pos_x instead.")
  (w_pos_x m))

(cl:ensure-generic-function 'w_pos_y-val :lambda-list '(m))
(cl:defmethod w_pos_y-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:w_pos_y-val is deprecated.  Use cat_move_to_target-srv:w_pos_y instead.")
  (w_pos_y m))

(cl:ensure-generic-function 'w_pos_z-val :lambda-list '(m))
(cl:defmethod w_pos_z-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:w_pos_z-val is deprecated.  Use cat_move_to_target-srv:w_pos_z instead.")
  (w_pos_z m))

(cl:ensure-generic-function 'foundtag-val :lambda-list '(m))
(cl:defmethod foundtag-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:foundtag-val is deprecated.  Use cat_move_to_target-srv:foundtag instead.")
  (foundtag m))

(cl:ensure-generic-function 'tag_id-val :lambda-list '(m))
(cl:defmethod tag_id-val ((m <GetTagPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cat_move_to_target-srv:tag_id-val is deprecated.  Use cat_move_to_target-srv:tag_id instead.")
  (tag_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTagPose-response>) ostream)
  "Serializes a message object of type '<GetTagPose-response>"
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ori_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ori_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ori_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ori_w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'w_pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'w_pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'w_pos_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'foundtag) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'tag_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTagPose-response>) istream)
  "Deserializes a message object of type '<GetTagPose-response>"
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
    (cl:setf (cl:slot-value msg 'ori_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori_w) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w_pos_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w_pos_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w_pos_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'foundtag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTagPose-response>)))
  "Returns string type for a service object of type '<GetTagPose-response>"
  "cat_move_to_target/GetTagPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTagPose-response)))
  "Returns string type for a service object of type 'GetTagPose-response"
  "cat_move_to_target/GetTagPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTagPose-response>)))
  "Returns md5sum for a message object of type '<GetTagPose-response>"
  "9947cf06bd814875243bead8e5c2eb9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTagPose-response)))
  "Returns md5sum for a message object of type 'GetTagPose-response"
  "9947cf06bd814875243bead8e5c2eb9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTagPose-response>)))
  "Returns full string definition for message of type '<GetTagPose-response>"
  (cl:format cl:nil "float64 pos_x~%float64 pos_y~%float64 pos_z~%float64 ori_x~%float64 ori_y~%float64 ori_z~%float64 ori_w~%float64 w_pos_x~%float64 w_pos_y~%float64 w_pos_z~%bool foundtag~%int16 tag_id~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTagPose-response)))
  "Returns full string definition for message of type 'GetTagPose-response"
  (cl:format cl:nil "float64 pos_x~%float64 pos_y~%float64 pos_z~%float64 ori_x~%float64 ori_y~%float64 ori_z~%float64 ori_w~%float64 w_pos_x~%float64 w_pos_y~%float64 w_pos_z~%bool foundtag~%int16 tag_id~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTagPose-response>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTagPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTagPose-response
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
    (cl:cons ':pos_z (pos_z msg))
    (cl:cons ':ori_x (ori_x msg))
    (cl:cons ':ori_y (ori_y msg))
    (cl:cons ':ori_z (ori_z msg))
    (cl:cons ':ori_w (ori_w msg))
    (cl:cons ':w_pos_x (w_pos_x msg))
    (cl:cons ':w_pos_y (w_pos_y msg))
    (cl:cons ':w_pos_z (w_pos_z msg))
    (cl:cons ':foundtag (foundtag msg))
    (cl:cons ':tag_id (tag_id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTagPose)))
  'GetTagPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTagPose)))
  'GetTagPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTagPose)))
  "Returns string type for a service object of type '<GetTagPose>"
  "cat_move_to_target/GetTagPose")