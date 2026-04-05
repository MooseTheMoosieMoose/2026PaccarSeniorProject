;; Auto-generated. Do not edit!


(when (boundp 'coop_per_msgs::ObjectDetection)
  (if (not (find-package "COOP_PER_MSGS"))
    (make-package "COOP_PER_MSGS"))
  (shadow 'ObjectDetection (find-package "COOP_PER_MSGS")))
(unless (find-package "COOP_PER_MSGS::OBJECTDETECTION")
  (make-package "COOP_PER_MSGS::OBJECTDETECTION"))

(in-package "ROS")
;;//! \htmlinclude ObjectDetection.msg.html


(defclass coop_per_msgs::ObjectDetection
  :super ros::object
  :slots (_latitude _longitude _altitude _length _width _height _rotation _timestamp _class _confidence ))

(defmethod coop_per_msgs::ObjectDetection
  (:init
   (&key
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:length __length) 0.0)
    ((:width __width) 0.0)
    ((:height __height) 0.0)
    ((:rotation __rotation) 0.0)
    ((:timestamp __timestamp) 0.0)
    ((:class __class) "")
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _length (float __length))
   (setq _width (float __width))
   (setq _height (float __height))
   (setq _rotation (float __rotation))
   (setq _timestamp (float __timestamp))
   (setq _class (string __class))
   (setq _confidence (float __confidence))
   self)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:rotation
   (&optional __rotation)
   (if __rotation (setq _rotation __rotation)) _rotation)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:class
   (&optional __class)
   (if __class (setq _class __class)) _class)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _altitude
    8
    ;; float32 _length
    4
    ;; float32 _width
    4
    ;; float32 _height
    4
    ;; float32 _rotation
    4
    ;; float64 _timestamp
    8
    ;; string _class
    4 (length _class)
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _length
       (sys::poke _length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotation
       (sys::poke _rotation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _class
       (write-long (length _class) s) (princ _class s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _length
     (setq _length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotation
     (setq _rotation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get coop_per_msgs::ObjectDetection :md5sum-) "54af25369463fc072d402f549fb57e71")
(setf (get coop_per_msgs::ObjectDetection :datatype-) "coop_per_msgs/ObjectDetection")
(setf (get coop_per_msgs::ObjectDetection :definition-)
      "#Position in Global Space
#Note that 32 bit floats have a worst case accuracy around ~1.7 meters
#While a 64 bit float has a worset case accuracy around ~3.16 nanometers
float64 latitude
float64 longitude
float64 altitude

#The objects dimensions in meters
float32 length
float32 width
float32 height

#The objects rotation along its up/down (Z) axis
float32 rotation

#The UNIX epoch timestamp
float64 timestamp

#The class name and its confidence score
string class
float32 confidence

")



(provide :coop_per_msgs/ObjectDetection "54af25369463fc072d402f549fb57e71")


