;; Auto-generated. Do not edit!


(when (boundp 'feature_recon::Persons)
  (if (not (find-package "FEATURE_RECON"))
    (make-package "FEATURE_RECON"))
  (shadow 'Persons (find-package "FEATURE_RECON")))
(unless (find-package "FEATURE_RECON::PERSONS")
  (make-package "FEATURE_RECON::PERSONS"))

(in-package "ROS")
;;//! \htmlinclude Persons.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass feature_recon::Persons
  :super ros::object
  :slots (_persons _image_w _image_h _header ))

(defmethod feature_recon::Persons
  (:init
   (&key
    ((:persons __persons) (let (r) (dotimes (i 0) (push (instance feature_recon::Person :init) r)) r))
    ((:image_w __image_w) 0)
    ((:image_h __image_h) 0)
    ((:header __header) (instance std_msgs::Header :init))
    )
   (send-super :init)
   (setq _persons __persons)
   (setq _image_w (round __image_w))
   (setq _image_h (round __image_h))
   (setq _header __header)
   self)
  (:persons
   (&rest __persons)
   (if (keywordp (car __persons))
       (send* _persons __persons)
     (progn
       (if __persons (setq _persons (car __persons)))
       _persons)))
  (:image_w
   (&optional __image_w)
   (if __image_w (setq _image_w __image_w)) _image_w)
  (:image_h
   (&optional __image_h)
   (if __image_h (setq _image_h __image_h)) _image_h)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:serialization-length
   ()
   (+
    ;; feature_recon/Person[] _persons
    (apply #'+ (send-all _persons :serialization-length)) 4
    ;; uint32 _image_w
    4
    ;; uint32 _image_h
    4
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; feature_recon/Person[] _persons
     (write-long (length _persons) s)
     (dolist (elem _persons)
       (send elem :serialize s)
       )
     ;; uint32 _image_w
       (write-long _image_w s)
     ;; uint32 _image_h
       (write-long _image_h s)
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; feature_recon/Person[] _persons
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _persons (let (r) (dotimes (i n) (push (instance feature_recon::Person :init) r)) r))
     (dolist (elem- _persons)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32 _image_w
     (setq _image_w (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _image_h
     (setq _image_h (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;;
   self)
  )

(setf (get feature_recon::Persons :md5sum-) "42b07b171c8bbbc4471c42a63942d54b")
(setf (get feature_recon::Persons :datatype-) "feature_recon/Persons")
(setf (get feature_recon::Persons :definition-)
      "Person[] persons
uint32 image_w
uint32 image_h
Header header
================================================================================
MSG: feature_recon/Person
BodyPartElm[] body_part
================================================================================
MSG: feature_recon/BodyPartElm
uint32 part_id
float32 x
float32 y
float32 z
float32 confidence

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :feature_recon/Persons "42b07b171c8bbbc4471c42a63942d54b")


