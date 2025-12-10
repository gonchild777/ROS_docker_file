;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::PointArray)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'PointArray (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::POINTARRAY")
  (make-package "CUSTOM_MSGS::POINTARRAY"))

(in-package "ROS")
;;//! \htmlinclude PointArray.msg.html


(defclass custom_msgs::PointArray
  :super ros::object
  :slots (_points ))

(defmethod custom_msgs::PointArray
  (:init
   (&key
    ((:points __points) ())
    )
   (send-super :init)
   (setq _points __points)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; custom_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; custom_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; custom_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance custom_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get custom_msgs::PointArray :md5sum-) "3fb3f9dacc279b964c4c8341122c34df")
(setf (get custom_msgs::PointArray :datatype-) "custom_msgs/PointArray")
(setf (get custom_msgs::PointArray :definition-)
      "Point[] points

================================================================================
MSG: custom_msgs/Point
float64 x
float64 y
float64 z
")



(provide :custom_msgs/PointArray "3fb3f9dacc279b964c4c8341122c34df")


