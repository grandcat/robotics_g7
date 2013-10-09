; Auto-generated. Do not edit!


(cl:in-package camera-msg)


;//! \htmlinclude Position.msg.html

(cl:defclass <Position> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:integer
    :initform 0))
)

(cl:defclass Position (<Position>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Position>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Position)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-msg:<Position> is deprecated: use camera-msg:Position instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:x-val is deprecated.  Use camera-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <Position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-msg:depth-val is deprecated.  Use camera-msg:depth instead.")
  (depth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Position>) ostream)
  "Serializes a message object of type '<Position>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'depth)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Position>) istream)
  "Deserializes a message object of type '<Position>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'depth) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Position>)))
  "Returns string type for a message object of type '<Position>"
  "camera/Position")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Position)))
  "Returns string type for a message object of type 'Position"
  "camera/Position")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Position>)))
  "Returns md5sum for a message object of type '<Position>"
  "19be51867ea54121b0358052b3ed7d11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Position)))
  "Returns md5sum for a message object of type 'Position"
  "19be51867ea54121b0358052b3ed7d11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Position>)))
  "Returns full string definition for message of type '<Position>"
  (cl:format cl:nil "int32 x~%int32 depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Position)))
  "Returns full string definition for message of type 'Position"
  (cl:format cl:nil "int32 x~%int32 depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Position>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Position>))
  "Converts a ROS message object to a list"
  (cl:list 'Position
    (cl:cons ':x (x msg))
    (cl:cons ':depth (depth msg))
))
