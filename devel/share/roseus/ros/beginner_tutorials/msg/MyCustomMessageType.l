;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::MyCustomMessageType)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'MyCustomMessageType (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::MYCUSTOMMESSAGETYPE")
  (make-package "BEGINNER_TUTORIALS::MYCUSTOMMESSAGETYPE"))

(in-package "ROS")
;;//! \htmlinclude MyCustomMessageType.msg.html


(defclass beginner_tutorials::MyCustomMessageType
  :super ros::object
  :slots (_isValid _restartsRemaining _encoderCounts _distanceToStart _distanceToEnd _taskDescription _distancesToOtherRobots ))

(defmethod beginner_tutorials::MyCustomMessageType
  (:init
   (&key
    ((:isValid __isValid) nil)
    ((:restartsRemaining __restartsRemaining) 0)
    ((:encoderCounts __encoderCounts) 0)
    ((:distanceToStart __distanceToStart) 0.0)
    ((:distanceToEnd __distanceToEnd) 0.0)
    ((:taskDescription __taskDescription) "")
    ((:distancesToOtherRobots __distancesToOtherRobots) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _isValid __isValid)
   (setq _restartsRemaining (round __restartsRemaining))
   (setq _encoderCounts (round __encoderCounts))
   (setq _distanceToStart (float __distanceToStart))
   (setq _distanceToEnd (float __distanceToEnd))
   (setq _taskDescription (string __taskDescription))
   (setq _distancesToOtherRobots __distancesToOtherRobots)
   self)
  (:isValid
   (&optional (__isValid :null))
   (if (not (eq __isValid :null)) (setq _isValid __isValid)) _isValid)
  (:restartsRemaining
   (&optional __restartsRemaining)
   (if __restartsRemaining (setq _restartsRemaining __restartsRemaining)) _restartsRemaining)
  (:encoderCounts
   (&optional __encoderCounts)
   (if __encoderCounts (setq _encoderCounts __encoderCounts)) _encoderCounts)
  (:distanceToStart
   (&optional __distanceToStart)
   (if __distanceToStart (setq _distanceToStart __distanceToStart)) _distanceToStart)
  (:distanceToEnd
   (&optional __distanceToEnd)
   (if __distanceToEnd (setq _distanceToEnd __distanceToEnd)) _distanceToEnd)
  (:taskDescription
   (&optional __taskDescription)
   (if __taskDescription (setq _taskDescription __taskDescription)) _taskDescription)
  (:distancesToOtherRobots
   (&optional __distancesToOtherRobots)
   (if __distancesToOtherRobots (setq _distancesToOtherRobots __distancesToOtherRobots)) _distancesToOtherRobots)
  (:serialization-length
   ()
   (+
    ;; bool _isValid
    1
    ;; uint32 _restartsRemaining
    4
    ;; int32 _encoderCounts
    4
    ;; float32 _distanceToStart
    4
    ;; float64 _distanceToEnd
    8
    ;; string _taskDescription
    4 (length _taskDescription)
    ;; float64[] _distancesToOtherRobots
    (* 8    (length _distancesToOtherRobots)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isValid
       (if _isValid (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _restartsRemaining
       (write-long _restartsRemaining s)
     ;; int32 _encoderCounts
       (write-long _encoderCounts s)
     ;; float32 _distanceToStart
       (sys::poke _distanceToStart (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _distanceToEnd
       (sys::poke _distanceToEnd (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _taskDescription
       (write-long (length _taskDescription) s) (princ _taskDescription s)
     ;; float64[] _distancesToOtherRobots
     (write-long (length _distancesToOtherRobots) s)
     (dotimes (i (length _distancesToOtherRobots))
       (sys::poke (elt _distancesToOtherRobots i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isValid
     (setq _isValid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _restartsRemaining
     (setq _restartsRemaining (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _encoderCounts
     (setq _encoderCounts (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _distanceToStart
     (setq _distanceToStart (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _distanceToEnd
     (setq _distanceToEnd (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _taskDescription
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _taskDescription (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[] _distancesToOtherRobots
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _distancesToOtherRobots (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _distancesToOtherRobots i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get beginner_tutorials::MyCustomMessageType :md5sum-) "669304755248d0e6a47460f571ab5053")
(setf (get beginner_tutorials::MyCustomMessageType :datatype-) "beginner_tutorials/MyCustomMessageType")
(setf (get beginner_tutorials::MyCustomMessageType :definition-)
      "bool isValid
uint32 restartsRemaining
int32 encoderCounts
float32 distanceToStart
float64 distanceToEnd
string taskDescription
float64[] distancesToOtherRobots
")



(provide :beginner_tutorials/MyCustomMessageType "669304755248d0e6a47460f571ab5053")


