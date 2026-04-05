#!/usr/bin/env python3.8

#===============================================================================================
                                                       
# ,-----.  ,---.  ,--.   ,--.,------.,------.   ,---.   
#'  .--./ /  O  \ |   `.'   ||  .---'|  .--. ' /  O  \  
#|  |    |  .-.  ||  |'.'|  ||  `--, |  '--'.'|  .-.  | 
#'  '--'\|  | |  ||  |   |  ||  `---.|  |\  \ |  | |  | 
# `-----'`--' `--'`--'   `--'`------'`--' '--'`--' `--' 

#The camera node is based on Ultralytic's YoloV8 and takes in raw camera data, passes it through
#The model, projects it to 3d space, then publishes it to the camera's detections channel
# Moose Abou-Harb and Jake Sleppy - 04/05/26
                                                       
#===============================================================================================

#Import typing for strong typing
from typing import *

#Import all the ROS things
import rospy
import message_filters
from sensor_msgs.msg import CameraInfo, Image
from coop_per_msgs.msg import ObjectDetection

#Import all the extra packages for image manipulation and inference
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
import cv2
import os

#Camera path Consts to get the camera intrinsicts and RGBD data
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
CAMERA_IMAGE_TOPIC = "/camera/color/image_raw"
CAMERA_DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
CAMERA_DETECTIONS_TOPIC = "detections"

#Instantiate the CVBridge to convert the ROS Images to easily manipulated CV2 images
bridge = CvBridge()

class Point3d:
    """
    A simple wrapper around a vec<3> equivalent

    Attributes:
        x: float - the X component
        y: float - the Y component
        z: float - the Z component
    """
    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self.x: float = x
        self.y: float = y
        self.z: float = z

class Point2d:
    """
    A simple wrapper around a vec<2> equivalent with a method to project into
    3D space

    Attributes:
        x: float - the X component
        y: float - the Y component
    """

    #The transform matrix that converts camera coords to woorld coords, see `project` for detals
    coord_conv_mat = np.array([
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0]
    ])

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def project(self, depth: float, camera_intrinsics) -> Point3d:
        """
        Given a point in pixel space from the camera, a depth value, and the cameras intrinsics, projects
        the point to world space, and converts from optical coords to ROS coords, where optical coords define
        x as right, y as down and z as forward, and ROS coords are x forward, y is left and z is up, i.e multiplication by
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]

        """
        fx, fy, cx, cy = camera_intrinsics
        x_ws = ((self.x - cx) * depth) / fx
        y_ws = ((self.y - cy) * depth) / fy
        ws_point = Point2d.coord_conv_mat.dot(np.array([x_ws, y_ws, depth]))
        return Point3d(ws_point[0], ws_point[1], ws_point[2])

class YoloV8CameraNode():
    """
    The camera node that takes in raw camera data and runs it through YoloV8 to produce final detections

    Attributes:
        last_image_rgb: Optional[np.ndarray] - the most recent raw RGB data from the camera
        last_image_depth: Optional[np.ndarray] - the most recent depth value from the camera in meters
        camera_intrinsics: Tuple[float, 4] - A tuple defined as (focal_x, focal_y, principle_x, principle_y)
    """
    def __init__(self,
        camera_info_topic: str, 
        camera_rgb_topic: str, 
        camera_depth_topic: str, 
        detection_publish_topic: str
    ) -> None:
        """
        Constructor for the camera node

        Args:
            camera_info_topic: str - the topic to get the camera intrinsics, K
            camera_rgb_topic: str - the topic to get the camera's rgb data
            camera_depth_topic: str - the topic to get the camera's depth / Z data
            detection_publish_topic - the topic to publish the detections we create to
        """
        #Save our topics
        self.camera_info_topic = camera_info_topic
        self.camera_rgb_topic = camera_rgb_topic
        self.camera_depth_topic = camera_depth_topic
        self.detection_publich_topic = self.detection_publich_topic

        #Create refs to store the most recently received data
        self.last_image_rgb: Optional[np.ndarray] = None
        self.last_image_depth: Optional[np.ndarray] = None

        #Try to silence Yolo, might not work, TODO figure out how to do this right
        os.environ['YOLO_VERBOSE'] = 'False'

        #Create the custom publisher for the object detection messages
        obj_pub = rospy.Publisher("detections", ObjectDetection, queue_size=10)

        #Declare our node
        rospy.init_node("camera_node", anonymous=False)

        #Wait on the camera node to be up and running, fetch its camera data
        cam_meta = self.__camera_info_blocking_callback()
        focal_x = cam_meta.P[0]
        focal_y = cam_meta.P[5]
        principle_x = cam_meta.P[2]
        principle_y = cam_meta.P[6]
        loginfo = f"Camera has focal lengths: {focal_x}, {focal_y}, with principle points: {principle_x}, {principle_y}"
        self.camera_intrinsics = (focal_x, focal_y, principle_x, principle_y)
        rospy.loginfo(loginfo)

        #Create a subscriber to bind to the camera topics
        rgb_sub = message_filters.Subscriber(self.camera_rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.camera_depth_topic, Image)
        message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], 
            queue_size=10,
            slop=0.05
        ).registerCallback(self.__camera_callback)

        #Create our YOLO instance
        self.model = YOLO("/workspace/yolov8s.pt")
        rospy.loginfo("YOLO Model loaded and ready!")

        #Enter our infinite processing loop
        self.process()

    def process(self) -> None:
        """
        An infinite loop that fetches the most recently received images and produces 
        the resulting detections from YOLO

        Args:
            None

        Returns:
            None
        """
        while not rospy.is_shutdown():

            #Ensure we have SOMETHING
            if ((self.last_image_rgb != None) and (self.last_image_depth != None)):

                #Capture a copy of the new frames, and reset class instances
                new_image: np.ndarray = self.last_image_rgb
                new_depth: np.ndarray = self.last_image_depth
                self.last_image_depth = None
                self.last_image_rgb = None

                #Run inference
                results = self.model.predict(new_image, verbose=False)

                #Loop over results
                for result in results:
                    class_names = result.names
                    for box in result.boxes:

                        #Get classification and confidence
                        class_id_tensor = box.cls
                        class_id = int(class_id_tensor.item())
                        class_name = class_names[class_id]
                        class_conf = box.conf.item()

                        #Get the base info on the boxes dimensions and depth
                        corner_coords = box.xyxy.cpu().numpy()[0]
                        xmin, ymin, xmax, ymax = map(int, corner_coords)
                        xmax_clamp = min(xmax, new_depth.shape[1] - 1)
                        ymax_clamp = min(ymax, new_depth.shape[0] - 1) 

                        #Create corners and center points, then project it into worldspace
                        bottom_corner_ws: Point3d = Point2d(xmin, ymin).project(new_depth[ymin][xmin], self.camera_intrinsics)
                        top_corner_ws: Point3d = Point2d(xmax, ymax).project(new_depth[ymax_clamp][xmax_clamp], self.camera_intrinsics)
                        center_point_ss = Point2d(xmin + ((xmax - xmin) / 2), ymin + ((ymax - ymin) / 2))
                        center_point_ws = center_point_ss.project(new_depth[int(center_point_ss.y)][int(center_point_ss.x)], self.camera_intrinsics)

                        #Calculate the 3d boxes dimensions, folding depth to create a 3d rectangular prism
                        #At this point, z is up down, y is left and right, and x is depth, this is standard ROS coords
                        near_plane_x = min(bottom_corner_ws.x, top_corner_ws.x, center_point_ws.x)
                        width = abs(top_corner_ws.y - bottom_corner_ws.y)
                        height = abs(top_corner_ws.z - bottom_corner_ws.z)
                        depth = width

                        #Calculate the center point of the box, by increasing its depth to half the rectangular prism
                        center_point_ws.x = near_plane_x + 0.5 * depth

                        #Publish the detection
                        det_msg = f"Detected Object\n \tClass: {class_name}\n\tConf: {class_conf}\n\tCenter: ({center_point_ws.x},{center_point_ws.y},{center_point_ws.z})\n\tSize: ({depth},{height},{width})\n"
                        rospy.loginfo(det_msg)
            else:
                #Sleep so that hopefully callbacks can run
                rospy.sleep(0.1)

    def __camera_info_blocking_callback(self) -> CameraInfo:
        """
        Blocking wait to receive camera intrinsic info, K, private class member

        Args:
            None

        Returns:
            CameraInfo - the camera info ROS sensor message, used to get the K matrix
        """
        try:
            cam_info = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=None)
            return cam_info
        except rospy.ROSInterruptException:
            rospy.logwarn("ROS Shutdown occured before camera node could fetch camera metadata!")

    def __camera_callback(self, rgb_data: Image, depth_data: Image) -> None:
        """
        Callback to receive the most recent message of the RGB and depth from camera

        Args:
            rgb_data: Image - the raw rgb image populated by the callback
            depth_data: Image - the raw depth values (uint16_t millimeters) populated by the callback
        
        Returns:
            None
        """

        #Get CV2 forms of the images for easy manipulation, note that the depth image is interpolated
        #and resized so that depth values overlay, with the depth given in MM conver to M
        self.last_image_rgb = bridge.imgmsg_to_cv2(rgb_data)
        self.last_image_depth = bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough").astype(np.float32) / 1000.0


if __name__ == "__main__":
    #Create our node and when it quits, spin
    YoloV8CameraNode(CAMERA_INFO_TOPIC, CAMERA_IMAGE_TOPIC, CAMERA_DEPTH_TOPIC, CAMERA_DETECTIONS_TOPIC)
    rospy.spin()