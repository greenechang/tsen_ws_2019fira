; Auto-generated. Do not edit!


(cl:in-package op3_gripper-msg)


;//! \htmlinclude GripperPosition.msg.html

(cl:defclass <GripperPosition> (roslisp-msg-protocol:ros-message)
  ((Left
    :reader Left
    :initarg :Left
    :type cl:integer
    :initform 0)
   (Right
    :reader Right
    :initarg :Right
    :type cl:integer
    :initform 0))
)

(cl:defclass GripperPosition (<GripperPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name op3_gripper-msg:<GripperPosition> is deprecated: use op3_gripper-msg:GripperPosition instead.")))

(cl:ensure-generic-function 'Left-val :lambda-list '(m))
(cl:defmethod Left-val ((m <GripperPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_gripper-msg:Left-val is deprecated.  Use op3_gripper-msg:Left instead.")
  (Left m))

(cl:ensure-generic-function 'Right-val :lambda-list '(m))
(cl:defmethod Right-val ((m <GripperPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader op3_gripper-msg:Right-val is deprecated.  Use op3_gripper-msg:Right instead.")
  (Right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperPosition>) ostream)
  "Serializes a message object of type '<GripperPosition>"
  (cl:let* ((signed (cl:slot-value msg 'Left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperPosition>) istream)
  "Deserializes a message object of type '<GripperPosition>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Left) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Right) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperPosition>)))
  "Returns string type for a message object of type '<GripperPosition>"
  "op3_gripper/GripperPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperPosition)))
  "Returns string type for a message object of type 'GripperPosition"
  "op3_gripper/GripperPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperPosition>)))
  "Returns md5sum for a message object of type '<GripperPosition>"
  "5b0e67f9a5c183da3139b32cb4f14d70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperPosition)))
  "Returns md5sum for a message object of type 'GripperPosition"
  "5b0e67f9a5c183da3139b32cb4f14d70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperPosition>)))
  "Returns full string definition for message of type '<GripperPosition>"
  (cl:format cl:nil "int64 Left~%int64 Right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperPosition)))
  "Returns full string definition for message of type 'GripperPosition"
  (cl:format cl:nil "int64 Left~%int64 Right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperPosition>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperPosition
    (cl:cons ':Left (Left msg))
    (cl:cons ':Right (Right msg))
))
