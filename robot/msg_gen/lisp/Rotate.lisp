; Auto-generated. Do not edit!


(cl:in-package robot-msg)


;//! \htmlinclude Rotate.msg.html

(cl:defclass <Rotate> (roslisp-msg-protocol:ros-message)
  ((right
    :reader right
    :initarg :right
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Rotate (<Rotate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rotate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rotate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot-msg:<Rotate> is deprecated: use robot-msg:Rotate instead.")))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <Rotate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot-msg:right-val is deprecated.  Use robot-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rotate>) ostream)
  "Serializes a message object of type '<Rotate>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rotate>) istream)
  "Deserializes a message object of type '<Rotate>"
    (cl:setf (cl:slot-value msg 'right) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rotate>)))
  "Returns string type for a message object of type '<Rotate>"
  "robot/Rotate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rotate)))
  "Returns string type for a message object of type 'Rotate"
  "robot/Rotate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rotate>)))
  "Returns md5sum for a message object of type '<Rotate>"
  "90337cd7fb14d26b7be15ee3df49f209")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rotate)))
  "Returns md5sum for a message object of type 'Rotate"
  "90337cd7fb14d26b7be15ee3df49f209")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rotate>)))
  "Returns full string definition for message of type '<Rotate>"
  (cl:format cl:nil "bool right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rotate)))
  "Returns full string definition for message of type 'Rotate"
  (cl:format cl:nil "bool right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rotate>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rotate>))
  "Converts a ROS message object to a list"
  (cl:list 'Rotate
    (cl:cons ':right (right msg))
))
