;; Auto-generated. Do not edit!


(when (boundp 'feature_recon::Person)
  (if (not (find-package "FEATURE_RECON"))
    (make-package "FEATURE_RECON"))
  (shadow 'Person (find-package "FEATURE_RECON")))
(unless (find-package "FEATURE_RECON::PERSON")
  (make-package "FEATURE_RECON::PERSON"))

(in-package "ROS")
;;//! \htmlinclude Person.msg.html


(defclass feature_recon::Person
  :super ros::object
  :slots (_body_part ))

(defmethod feature_recon::Person
  (:init
   (&key
    ((:body_part __body_part) (let (r) (dotimes (i 0) (push (instance feature_recon::BodyPartElm :init) r)) r))
    )
   (send-super :init)
   (setq _body_part __body_part)
   self)
  (:body_part
   (&rest __body_part)
   (if (keywordp (car __body_part))
       (send* _body_part __body_part)
     (progn
       (if __body_part (setq _body_part (car __body_part)))
       _body_part)))
  (:serialization-length
   ()
   (+
    ;; feature_recon/BodyPartElm[] _body_part
    (apply #'+ (send-all _body_part :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; feature_recon/BodyPartElm[] _body_part
     (write-long (length _body_part) s)
     (dolist (elem _body_part)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; feature_recon/BodyPartElm[] _body_part
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _body_part (let (r) (dotimes (i n) (push (instance feature_recon::BodyPartElm :init) r)) r))
     (dolist (elem- _body_part)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get feature_recon::Person :md5sum-) "71df5a728503c3e077184dbd734a3f82")
(setf (get feature_recon::Person :datatype-) "feature_recon/Person")
(setf (get feature_recon::Person :definition-)
      "BodyPartElm[] body_part
================================================================================
MSG: feature_recon/BodyPartElm
uint32 part_id
float32 x
float32 y
float32 z
float32 confidence

")



(provide :feature_recon/Person "71df5a728503c3e077184dbd734a3f82")


