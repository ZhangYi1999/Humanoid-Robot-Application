;; Auto-generated. Do not edit!


(when (boundp 'control::MoveJoints)
  (if (not (find-package "CONTROL"))
    (make-package "CONTROL"))
  (shadow 'MoveJoints (find-package "CONTROL")))
(unless (find-package "CONTROL::MOVEJOINTS")
  (make-package "CONTROL::MOVEJOINTS"))
(unless (find-package "CONTROL::MOVEJOINTSREQUEST")
  (make-package "CONTROL::MOVEJOINTSREQUEST"))
(unless (find-package "CONTROL::MOVEJOINTSRESPONSE")
  (make-package "CONTROL::MOVEJOINTSRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass control::MoveJointsRequest
  :super ros::object
  :slots (_task _name _mask _target _max_speed _time _flag ))

(defmethod control::MoveJointsRequest
  (:init
   (&key
    ((:task __task) 0)
    ((:name __name) "")
    ((:mask __mask) 0.0)
    ((:target __target) (instance geometry_msgs::Twist :init))
    ((:max_speed __max_speed) 0.0)
    ((:time __time) 0.0)
    ((:flag __flag) nil)
    )
   (send-super :init)
   (setq _task (round __task))
   (setq _name (string __name))
   (setq _mask (float __mask))
   (setq _target __target)
   (setq _max_speed (float __max_speed))
   (setq _time (float __time))
   (setq _flag __flag)
   self)
  (:task
   (&optional __task)
   (if __task (setq _task __task)) _task)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:mask
   (&optional __mask)
   (if __mask (setq _mask __mask)) _mask)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:max_speed
   (&optional __max_speed)
   (if __max_speed (setq _max_speed __max_speed)) _max_speed)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; int8 _task
    1
    ;; string _name
    4 (length _name)
    ;; float32 _mask
    4
    ;; geometry_msgs/Twist _target
    (send _target :serialization-length)
    ;; float32 _max_speed
    4
    ;; float32 _time
    4
    ;; bool _flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _task
       (write-byte _task s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float32 _mask
       (sys::poke _mask (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Twist _target
       (send _target :serialize s)
     ;; float32 _max_speed
       (sys::poke _max_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _time
       (sys::poke _time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _task
     (setq _task (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _task 127) (setq _task (- _task 256)))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _mask
     (setq _mask (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Twist _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;; float32 _max_speed
     (setq _max_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _time
     (setq _time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass control::MoveJointsResponse
  :super ros::object
  :slots (_current ))

(defmethod control::MoveJointsResponse
  (:init
   (&key
    ((:current __current) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _current __current)
   self)
  (:current
   (&rest __current)
   (if (keywordp (car __current))
       (send* _current __current)
     (progn
       (if __current (setq _current (car __current)))
       _current)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Twist _current
    (send _current :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Twist _current
       (send _current :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Twist _current
     (send _current :deserialize buf ptr-) (incf ptr- (send _current :serialization-length))
   ;;
   self)
  )

(defclass control::MoveJoints
  :super ros::object
  :slots ())

(setf (get control::MoveJoints :md5sum-) "6637c617ed353a69ab1f189297893290")
(setf (get control::MoveJoints :datatype-) "control/MoveJoints")
(setf (get control::MoveJoints :request) control::MoveJointsRequest)
(setf (get control::MoveJoints :response) control::MoveJointsResponse)

(defmethod control::MoveJointsRequest
  (:response () (instance control::MoveJointsResponse :init)))

(setf (get control::MoveJointsRequest :md5sum-) "6637c617ed353a69ab1f189297893290")
(setf (get control::MoveJointsRequest :datatype-) "control/MoveJointsRequest")
(setf (get control::MoveJointsRequest :definition-)
      "
int8 task
string name

float32 mask
geometry_msgs/Twist target

float32 max_speed
float32 time

bool flag

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---

geometry_msgs/Twist current

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
")

(setf (get control::MoveJointsResponse :md5sum-) "6637c617ed353a69ab1f189297893290")
(setf (get control::MoveJointsResponse :datatype-) "control/MoveJointsResponse")
(setf (get control::MoveJointsResponse :definition-)
      "
int8 task
string name

float32 mask
geometry_msgs/Twist target

float32 max_speed
float32 time

bool flag

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---

geometry_msgs/Twist current

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
")



(provide :control/MoveJoints "6637c617ed353a69ab1f189297893290")


