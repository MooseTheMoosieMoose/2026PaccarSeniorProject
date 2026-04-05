#!/usr/bin/env python3

#===============================================================================================

#,--.   ,--.,------.    ,---.  ,------.  
#|  |   |  ||  .-.  \  /  O  \ |  .--. ' 
#|  |   |  ||  |  \  :|  .-.  ||  '--'.' 
#|  '--.|  ||  '--'  /|  | |  ||  |\  \  
#`-----'`--'`-------' `--' `--'`--' '--'                                        

# This is the main node for the LIDAR object detection, utilizing PointPillars, a simple but
# Effective model based around BEV projections from pillars, its a LIDAR only model which
# greatly decreases coupling over older VirConv based systems, and still runs zoomy like
# - Moose Abou-Harb 04/05/26

#===============================================================================================

#We want to enforce strong typing across the whole project
from typing import *

#For universal timing we pass the unix epoch timestamp
import time

#Import all the ros things for logging, pub/sub and message pack/unpacking
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

#Import our custom messages used between nodes
from coop_per_msgs.msg import ObjectDetection, ObjectDetectionFrame

#Use torch to run the PP model
import torch
import numpy as np
from pointpillars.model import PointPillars

#The .pth given to us by the repo
MODEL_STATE = "/opt/pointpillars/PointPillars/pretrained/epoch_160.pth"

# ROS subscriber topics
POINTCLOUD_TOPIC = "/ouster/points"

#Detection Publish topic
DETECTION_PUBLISH_TOPIC = "lidar_node/detections"


class PointPillarsLidarNode:
    """
    A ROS1 Node that implements the PointPillars model with live LIDAR data streamd as a PointCloud2 message,
    publishes its detections as a CoopPer ObjectDetection message

    Attributes:
        pointcloud_topic: str - the topic to receive point clouds on
        detection_publish_topic: str - the topic to publish detections on
        model_state_path: str - a file path to load the PointPillars model state from

        last_point_cloud: PointCloud2 - the most recent point cloud received, a private member
        model: PointPillars - the actual nn that points are passed through
        obj_pub - the publisher for our detection messages
    """

    #A quick map to correspond label indecies with their corresponding class
    class_label_map = {
        0 : "Pedestrian",
        1 : "Cyclist",
        2 : "Car"
    }

    def __init__(self, pointcloud_topic: str, detection_publish_topic: str, model_state_path: str):

        #Last point cloud stores the class ref to the last received point cloud
        self.last_point_cloud: Optional[np.ndarray] = None

        #Declare our ros node, then subscribe it to the point cloud publisher
        rospy.init_node("lidar_node", anonymous=False)
        rospy.Subscriber(pointcloud_topic, PointCloud2, self.__pointcloud_callback)

        #Create the custom publisher for the object detection messages
        self.obj_pub = rospy.Publisher(detection_publish_topic, ObjectDetectionFrame, queue_size=10)

        #Create our model
        rospy.loginfo("Creating model...")
        self.model = PointPillars(nclasses=3).cuda()
        rospy.loginfo("Model built! Loading state...")
        self.model.load_state_dict(torch.load(model_state_path))
        rospy.loginfo("State loaded! Model is good to go...")
        self.model.eval()

        #Enter the inference loop until shutdown
        self.process()

    def process(self) -> None:
        """
        An infinite processing loop where it waits for a valid new point cloud, passes it through PointPillars,
        then produces bounding boxes that are published accordingly

        Args:
            None
        """
        frame_id = 0
        while not rospy.is_shutdown():
            #Grab the current cloud and check if inference needs to be run
            if isinstance(self.last_point_cloud, np.ndarray) and self.last_point_cloud.shape[0] != 0:

                #Timestamp for Epoch time
                process_begin_time = time.time()

                #Convert the np array of the point cloud into torch tensors
                cloud_tensors = torch.from_numpy(self.last_point_cloud).cuda()
                self.last_point_cloud = None

                try:
                    with torch.no_grad():
                        #Forward the model with our tensors
                        res = self.model(batched_pts=[cloud_tensors], mode="test")[0]

                        #Loop over the results and create new detection messages
                        detections: List[ObjectDetection] = []
                        for indx, label_indx in enumerate(res["labels"]):
                            class_name = self.class_label_map[label_indx]
                            class_conf = res["scores"][indx]
                            bounding_box = res["lidar_bboxes"][indx]

                            new_detection: ObjectDetection = ObjectDetection()
                            new_detection.long_x = bounding_box[0]
                            new_detection.lat_y = bounding_box[1]
                            new_detection.altitude_z = bounding_box[2]

                            new_detection.width = bounding_box[3]
                            new_detection.depth = bounding_box[4]
                            new_detection.height = bounding_box[5]

                            new_detection.rotation = bounding_box[6]

                            new_detection.class_name = class_name
                            new_detection.confidence = class_conf 

                            detections.append(new_detection)

                        #Now that we have populated the detections we can publish them
                        new_detection_frame: ObjectDetectionFrame = ObjectDetectionFrame()
                        new_detection_frame.header.stamp = rospy.Time.now()
                        new_detection_frame.header.frame_id = frame_id
                        new_detection_frame.epoch_timestamp = process_begin_time
                        new_detection_frame.detections = detections
                        self.obj_pub.publish(new_detection_frame)
                        frame_id += 1

                        count = len(detections)
                        rospy.loginfo(f"New detection frame published from LIDAR, with {count} objects")

                except Exception as e:
                    rospy.loginfo(f"Something went wrong: {e}")
                
            rospy.sleep(0.1)


    def __pointcloud_callback(self, points: PointCloud2) -> None:
        """
        A private class callback that is the callback to fetch pointclouds.
        Populates the last_point_cloud class member

        Args:
            points: PointCloud2 - the cloud populated by a PC2 publisher
        
        Returns:
            None
        """

        #Convert ROS PointCloud2 to numpy
        points_list_gen = pc2.read_points(
            points,
            field_names=("x", "y", "z", "reflectivity"),
            skip_nans=True
        )
        points_list = np.array(list(points_list_gen), dtype=np.float32)

        #Clamp the intensity (reflectivity) values to the kitty standard range of 0->1
        points_list[:, 3] = np.clip(points_list[:, 3] / 255.0, 0.0, 1.0)
        self.last_point_cloud = np.array(points_list, dtype=np.float32).reshape(-1, 4)

if __name__ == "__main__":
    #Create the LIDAR Node, then spin on exit
    PointPillarsLidarNode(POINTCLOUD_TOPIC, DETECTION_PUBLISH_TOPIC, MODEL_STATE)
    rospy.spin()