; Auto-generated. Do not edit!


(cl:in-package robot-msg)


;//! \htmlinclude EKF.msg.html

(cl:defclass <EKF> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (y_wall
    :reader y_wall
    :initarg :y_wall
    :type cl:float
    :initform 0.0))
)

(cl:defclass EKF (<EKF>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EKF>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EKF)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot-msg:<EKF> is deprecated: use robot-msg:EKF instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <EKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot-msg:x-val is deprecated.  Use robot-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <EKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot-msg:y-val is deprecated.  Use robot-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <EKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot-msg:theta-val is deprecated.  Use robot-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'y_wall-val :lambda-list '(m))
(cl:defmethod y_wall-val ((m <EKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot-msg:y_wall-val is deprecated.  Use robot-msg:y_wall instead.")
  (y_wall m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EKF>) ostream)
  "Serializes a message object of type '<EKF>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_wall))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EKF>) istream)
  "Deserializes a message object of type '<EKF>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_wall) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EKF>)))
  "Returns string type for a message object of type '<EKF>"
  "robot/EKF")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EKF)))
  "Returns string type for a message object of type 'EKF"
  "robot/EKF")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EKF>)))
  "Returns md5sum for a message object of type '<EKF>"
  "5d9ce42a66b2094acde32a1b566e7ecd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EKF)))
  "Returns md5sum for a message object of type 'EKF"
  "5d9ce42a66b2094acde32a1b566e7ecd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EKF>)))
  "Returns full string definition for message of type '<EKF>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 theta~%float32 y_wall~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EKF)))
  "Returns full string definition for message of type 'EKF"
  (cl:format cl:nil "float32 x~%float32 y~%float32 theta~%float32 y_wall~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EKF>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EKF>))
  "Converts a ROS message object to a list"
  (cl:list 'EKF
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':y_wall (y_wall msg))
))
