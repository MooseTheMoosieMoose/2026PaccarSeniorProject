#!/usr/bin/env python3.8

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

#Instantiate the CVBridge to convert the ROS Images to easily manipulated CV2 images
bridge = CvBridge()

#The transform matrix that converts camera coords to woorld coords, see `project_to_ws` for detals
coord_conv_mat = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0]
])

#Keep globals that the callback will populate
#TODO depth is given in millimeters, see if this messes up WS projection
last_image_rgb = None
last_image_depth = None

class Point3d:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Point2d:
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

        :param: camera_point is a 2d tuple with [0] being x, and [1] being y
        :param: depth is the depth value at camera_point
        :param: camera_intrinsics is a 4d tuple, with (focal_x, focal_y, principal_x, principal_y)
        :returns: a 3d numpy array containing the point in worldspace
        """
        fx, fy, cx, cy = camera_intrinsics
        x_ws = ((self.x - cx) * depth) / fx
        y_ws = ((self.y - cy) * depth) / fy
        ws_point = coord_conv_mat.dot(np.array([x_ws, y_ws, depth]))
        return Point3d(ws_point[0], ws_point[1], ws_point[2])

def fetch_camera_info() -> CameraInfo:
    """
    Blocking wait to receive camera intrinsic info

    :returns: a CameraInfo message with the cameras intrinsics
    :rtype: CameraInfo
    """
    try:
        cam_info = rospy.wait_for_message(CAMERA_INFO_TOPIC, CameraInfo, timeout=None)
        return cam_info
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS Shutdown occured before camera node could fetch camera metadata!")

def camera_callback(rgb_data: Image, depth_data: Image) -> None:
    """
    Callback to receive the most recent message of the RGB from camera
    """
    global last_image_rgb, last_image_depth
    last_image_rgb = rgb_data
    last_image_depth = depth_data


def main():
    #Ref our last image
    global last_image_depth, last_image_rgb

    #Make yolo shut up
    os.environ['YOLO_VERBOSE'] = 'False' 

    #Create the custom publisher for the object detection messages
    obj_pub = rospy.Publisher("detections", ObjectDetection, queue_size=10)

    #Declare our node
    rospy.init_node("camera_inference_node", anonymous=False)

    #Wait on the camera node to be up and running, fetch its camera data
    cam_meta = fetch_camera_info()
    focal_x = cam_meta.P[0]
    focal_y = cam_meta.P[5]
    principle_x = cam_meta.P[2]
    principle_y = cam_meta.P[6]
    loginfo = f"Camera has focal lengths: {focal_x}, {focal_y}, with principle points: {principle_x}, {principle_y}"
    camera_intrinsics = (focal_x, focal_y, principle_x, principle_y)
    rospy.loginfo(loginfo)

    #Create a subscriber to bind to the camera topics
    rgb_sub = message_filters.Subscriber(CAMERA_IMAGE_TOPIC, Image)
    depth_sub = message_filters.Subscriber(CAMERA_DEPTH_TOPIC, Image)
    message_filters.ApproximateTimeSynchronizer(
        [rgb_sub, depth_sub], 
        queue_size=10,
        slop=0.05
    ).registerCallback(camera_callback)

    #Load up YOLO
    model = YOLO("/workspace/yolov8s.pt")
    rospy.loginfo("YOLO Model loaded and ready!")

    #Loop and process
    while not rospy.is_shutdown():

        if ((last_image_rgb != None) and (last_image_depth != None)):

            #Get CV2 forms of the images for easy manipulation, note that the depth image is interpolated
            #and resized so that depth values overlay, with the depth given in MM conver to M
            cv_rgb_image = bridge.imgmsg_to_cv2(last_image_rgb)
            cv_depth_raw = bridge.imgmsg_to_cv2(last_image_depth, desired_encoding="passthrough")
            cv_depth_image = cv_depth_raw.astype(np.float32) / 1000.0

            # rgb_height, rgb_width = cv_rgb_image.shape[:2]
            # depth_height, depth_width = cv_depth_image.shape[:2] 
            # rospy.loginfo(f"RGB: ({rgb_height}, {rgb_width}), Depth: ({depth_height}, {depth_width})")

            #Reset so we can repopulate
            last_image_rgb = None
            last_image_depth = None

            #Run inference
            results = model.predict(cv_rgb_image, verbose=False)

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
                    xmax_clamp = min(xmax, cv_depth_image.shape[1] - 1)
                    ymax_clamp = min(ymax, cv_depth_image.shape[0] - 1) 

                    #Create corners and center points, then project it into worldspace
                    bottom_corner_ws: Point3d = Point2d(xmin, ymin).project(cv_depth_image[ymin][xmin], camera_intrinsics)
                    top_corner_ws: Point3d = Point2d(xmax, ymax).project(cv_depth_image[ymax_clamp][xmax_clamp], camera_intrinsics)
                    center_point_ss = Point2d(xmin + ((xmax - xmin) / 2), ymin + ((ymax - ymin) / 2))
                    center_point_ws = center_point_ss.project(cv_depth_image[int(center_point_ss.y)][int(center_point_ss.x)], camera_intrinsics)

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



    rospy.spin()

if __name__ == "__main__":
    main()