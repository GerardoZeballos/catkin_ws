; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude MyCustomMessageType.msg.html

(cl:defclass <MyCustomMessageType> (roslisp-msg-protocol:ros-message)
  ((isValid
    :reader isValid
    :initarg :isValid
    :type cl:boolean
    :initform cl:nil)
   (restartsRemaining
    :reader restartsRemaining
    :initarg :restartsRemaining
    :type cl:integer
    :initform 0)
   (encoderCounts
    :reader encoderCounts
    :initarg :encoderCounts
    :type cl:integer
    :initform 0)
   (distanceToStart
    :reader distanceToStart
    :initarg :distanceToStart
    :type cl:float
    :initform 0.0)
   (distanceToEnd
    :reader distanceToEnd
    :initarg :distanceToEnd
    :type cl:float
    :initform 0.0)
   (taskDescription
    :reader taskDescription
    :initarg :taskDescription
    :type cl:string
    :initform "")
   (distancesToOtherRobots
    :reader distancesToOtherRobots
    :initarg :distancesToOtherRobots
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MyCustomMessageType (<MyCustomMessageType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MyCustomMessageType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MyCustomMessageType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<MyCustomMessageType> is deprecated: use beginner_tutorials-msg:MyCustomMessageType instead.")))

(cl:ensure-generic-function 'isValid-val :lambda-list '(m))
(cl:defmethod isValid-val ((m <MyCustomMessageType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:isValid-val is deprecated.  Use beginner_tutorials-msg:isValid instead.")
  (isValid m))

(cl:ensure-generic-function 'restartsRemaining-val :lambda-list '(m))
(cl:defmethod restartsRemaining-val ((m <MyCustomMessageType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:restartsRemaining-val is deprecated.  Use beginner_tutorials-msg:restartsRemaining instead.")
  (restartsRemaining m))

(cl:ensure-generic-function 'encoderCounts-val :lambda-list '(m))
(cl:defmethod encoderCounts-val ((m <MyCustomMessageType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:encoderCounts-val is deprecated.  Use beginner_tutorials-msg:encoderCounts instead.")
  (encoderCounts m))

(cl:ensure-generic-function 'distanceToStart-val :lambda-list '(m))
(cl:defmethod distanceToStart-val ((m <MyCustomMessageType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:distanceToStart-val is deprecated.  Use beginner_tutorials-msg:distanceToStart instead.")
  (distanceToStart m))

(cl:ensure-generic-function 'distanceToEnd-val :lambda-list '(m))
(cl:defmethod distanceToEnd-val ((m <MyCustomMessageType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:distanceToEnd-val is deprecated.  Use beginner_tutorials-msg:distanceToEnd instead.")
  (distanceToEnd m))

(cl:ensure-generic-function 'taskDescription-val :lambda-list '(m))
(cl:defmethod taskDescription-val ((m <MyCustomMessageType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:taskDescription-val is deprecated.  Use beginner_tutorials-msg:taskDescription instead.")
  (taskDescription m))

(cl:ensure-generic-function 'distancesToOtherRobots-val :lambda-list '(m))
(cl:defmethod distancesToOtherRobots-val ((m <MyCustomMessageType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:distancesToOtherRobots-val is deprecated.  Use beginner_tutorials-msg:distancesToOtherRobots instead.")
  (distancesToOtherRobots m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MyCustomMessageType>) ostream)
  "Serializes a message object of type '<MyCustomMessageType>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isValid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'restartsRemaining)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'restartsRemaining)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'restartsRemaining)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'restartsRemaining)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'encoderCounts)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distanceToStart))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distanceToEnd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'taskDescription))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'taskDescription))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'distancesToOtherRobots))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'distancesToOtherRobots))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MyCustomMessageType>) istream)
  "Deserializes a message object of type '<MyCustomMessageType>"
    (cl:setf (cl:slot-value msg 'isValid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'restartsRemaining)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'restartsRemaining)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'restartsRemaining)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'restartsRemaining)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'encoderCounts) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distanceToStart) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distanceToEnd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'taskDescription) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'taskDescription) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'distancesToOtherRobots) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'distancesToOtherRobots)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MyCustomMessageType>)))
  "Returns string type for a message object of type '<MyCustomMessageType>"
  "beginner_tutorials/MyCustomMessageType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyCustomMessageType)))
  "Returns string type for a message object of type 'MyCustomMessageType"
  "beginner_tutorials/MyCustomMessageType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MyCustomMessageType>)))
  "Returns md5sum for a message object of type '<MyCustomMessageType>"
  "669304755248d0e6a47460f571ab5053")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MyCustomMessageType)))
  "Returns md5sum for a message object of type 'MyCustomMessageType"
  "669304755248d0e6a47460f571ab5053")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MyCustomMessageType>)))
  "Returns full string definition for message of type '<MyCustomMessageType>"
  (cl:format cl:nil "bool isValid~%uint32 restartsRemaining~%int32 encoderCounts~%float32 distanceToStart~%float64 distanceToEnd~%string taskDescription~%float64[] distancesToOtherRobots~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MyCustomMessageType)))
  "Returns full string definition for message of type 'MyCustomMessageType"
  (cl:format cl:nil "bool isValid~%uint32 restartsRemaining~%int32 encoderCounts~%float32 distanceToStart~%float64 distanceToEnd~%string taskDescription~%float64[] distancesToOtherRobots~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MyCustomMessageType>))
  (cl:+ 0
     1
     4
     4
     4
     8
     4 (cl:length (cl:slot-value msg 'taskDescription))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'distancesToOtherRobots) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MyCustomMessageType>))
  "Converts a ROS message object to a list"
  (cl:list 'MyCustomMessageType
    (cl:cons ':isValid (isValid msg))
    (cl:cons ':restartsRemaining (restartsRemaining msg))
    (cl:cons ':encoderCounts (encoderCounts msg))
    (cl:cons ':distanceToStart (distanceToStart msg))
    (cl:cons ':distanceToEnd (distanceToEnd msg))
    (cl:cons ':taskDescription (taskDescription msg))
    (cl:cons ':distancesToOtherRobots (distancesToOtherRobots msg))
))
