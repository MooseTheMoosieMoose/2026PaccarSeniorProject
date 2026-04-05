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

#Import all the ros things for logging, pub/sub and message pack/unpacking
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

#Use torch to run the PP model
import torch
import numpy as np
from pointpillars.model import PointPillars

#The .pth given to us by the repo
MODEL_STATE = "/opt/pointpillars/PointPillars/pretrained/epoch_160.pth"

# ROS subscriber topics
POINTCLOUD_TOPIC = "/ouster/points"

#Detection Publish topic
DETECTION_PUBLISH_TOPIC = "detections"


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
    """
    def __init__(self, pointcloud_topic: str, detection_publish_topic: str, model_state_path: str):

        #Last point cloud stores the class ref to the last received point cloud
        self.last_point_cloud: Optional[np.ndarray] = None

        #Declare our ros node, then subscribe it to the point cloud publisher
        rospy.init_node("lidar_node", anonymous=False)
        rospy.Subscriber(pointcloud_topic, PointCloud2, self.__pointcloud_callback)

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
        while not rospy.is_shutdown():
            #Grab the current cloud and check if inference needs to be run
            if isinstance(self.last_point_cloud, np.ndarray) and self.last_point_cloud.shape[0] != 0:

                #Convert the np array of the point cloud into torch tensors
                cloud_tensors = torch.from_numpy(self.last_point_cloud).cuda()
                self.last_point_cloud = None

                with torch.no_grad():
                    res = self.model(batched_pts=[cloud_tensors], mode="test")
                    rospy.loginfo(f"{res}")
                
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