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
  :slots (_lat_y _long_x _altitude_z _depth _width _height _rotation _class_name _confidence ))

(defmethod coop_per_msgs::ObjectDetection
  (:init
   (&key
    ((:lat_y __lat_y) 0.0)
    ((:long_x __long_x) 0.0)
    ((:altitude_z __altitude_z) 0.0)
    ((:depth __depth) 0.0)
    ((:width __width) 0.0)
    ((:height __height) 0.0)
    ((:rotation __rotation) 0.0)
    ((:class_name __class_name) "")
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _lat_y (float __lat_y))
   (setq _long_x (float __long_x))
   (setq _altitude_z (float __altitude_z))
   (setq _depth (float __depth))
   (setq _width (float __width))
   (setq _height (float __height))
   (setq _rotation (float __rotation))
   (setq _class_name (string __class_name))
   (setq _confidence (float __confidence))
   self)
  (:lat_y
   (&optional __lat_y)
   (if __lat_y (setq _lat_y __lat_y)) _lat_y)
  (:long_x
   (&optional __long_x)
   (if __long_x (setq _long_x __long_x)) _long_x)
  (:altitude_z
   (&optional __altitude_z)
   (if __altitude_z (setq _altitude_z __altitude_z)) _altitude_z)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:rotation
   (&optional __rotation)
   (if __rotation (setq _rotation __rotation)) _rotation)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; float64 _lat_y
    8
    ;; float64 _long_x
    8
    ;; float64 _altitude_z
    8
    ;; float32 _depth
    4
    ;; float32 _width
    4
    ;; float32 _height
    4
    ;; float32 _rotation
    4
    ;; string _class_name
    4 (length _class_name)
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _lat_y
       (sys::poke _lat_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _long_x
       (sys::poke _long_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude_z
       (sys::poke _altitude_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotation
       (sys::poke _rotation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _lat_y
     (setq _lat_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _long_x
     (setq _long_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude_z
     (setq _altitude_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _depth
     (setq _depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotation
     (setq _rotation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get coop_per_msgs::ObjectDetection :md5sum-) "2b10fb12cf3dbf0187359d2e5baae524")
(setf (get coop_per_msgs::ObjectDetection :datatype-) "coop_per_msgs/ObjectDetection")
(setf (get coop_per_msgs::ObjectDetection :definition-)
      "#Position in Global Space
#Note that 32 bit floats have a worst case accuracy around ~1.7 meters
#While a 64 bit float has a worset case accuracy around ~3.16 nanometers
float64 lat_y
float64 long_x
float64 altitude_z

#The objects dimensions in meters
float32 depth
float32 width
float32 height

#The objects rotation along its up/down (Z) axis
float32 rotation

#The class name and its confidence score
string class_name
float32 confidence

")



(provide :coop_per_msgs/ObjectDetection "2b10fb12cf3dbf0187359d2e5baae524")


