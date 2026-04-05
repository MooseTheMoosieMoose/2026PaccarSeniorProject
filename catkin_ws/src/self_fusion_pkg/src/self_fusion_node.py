#!/usr/bin/env python3.8

#===============================================================================================
                                                                                                                         
# ,---.         ,--. ,---.    ,------.               ,--.                
#'   .-'  ,---. |  |/  .-'    |  .---',--.,--. ,---. `--' ,---. ,--,--,  
#`.  `-. | .-. :|  ||  `-,    |  `--, |  ||  |(  .-' ,--.| .-. ||      \ 
#.-'    |\   --.|  ||  .-'    |  |`   '  ''  '.-'  `)|  |' '-' '|  ||  | 
#`-----'  `----'`--'`--'      `--'     `----' `----' `--' `---' `--''--' 

#The fusion node will listen to the camera and lidar detections, attempt to make a fusion of them,
#Then produce a new ObjectDetectionFrame that holds all the resulting objects, this will utilize
#The custom C++ fusion library thats pulled in from the container file
# - Moose Abou-Harb 04/05/26
#===============================================================================================

#Use strong typing
from typing import *

#Grab ros
import rospy
import message_filters

#Get our custom message types
from coop_per_msgs import ObjectDetection, ObjectDetectionFrame

#Detection sync slop defines the amount of slop allowed in fusable detection frames
MAX_FUSION_SYNC_SLOP = 0.1

#The topics for our source data and the topic that the fused results are pushed to
LIDAR_DETECTIONS_TOPIC = "lidar_node/detections"
CAMERA_DETECTIONS_TOPIC = "camera_node/detections"
FUSION_PUB_TOPIC = "self_fusion_node/fused"

class SelfFusionNode():
    def __init__(self, lidar_topic: str, camera_topic: str, pub_topic: str):
        #Save our paths
        self.lidar_topic = lidar_topic
        self.camera_topic = camera_topic
        self.pub_topic = pub_topic

        #Create class refs where our data callbacks can populate
        self.last_detection_frames: Optional[Dict[str, ObjectDetectionFrame]] = None

        #We create a time sync so that the detections we get are roughly synced with a specified slop value
        lidar_sub = message_filters.Subscriber(self.lidar_topic, ObjectDetectionFrame)
        camera_sub = message_filters.Subscriber(self.camera_topic, ObjectDetectionFrame)
        message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, camera_sub], 
            queue_size=10,
            slop=MAX_FUSION_SYNC_SLOP
        ).registerCallback(self.__detection_callback)

        #Create the custom publisher for the object detection messages
        self.obj_pub = rospy.Publisher(self.pub_topic, ObjectDetectionFrame, queue_size=10)

    def __detection_callback(self, camera_df: ObjectDetectionFrame, lidar_df: ObjectDetectionFrame) -> None:
        self.last_detection_frames = {"camera" : camera_df, "lidar" : lidar_df}
        camera_count = len(camera_df.detections)
        lidar_count = len(lidar_df.detections)
        rospy.loginfo(f"Synced Detections picked up: Camera -> {camera_count}, Lidar -> {lidar_count}")

if __name__ == "__main__":
    SelfFusionNode(LIDAR_DETECTIONS_TOPIC, CAMERA_DETECTIONS_TOPIC, FUSION_PUB_TOPIC)
    rospy.spin()