#!/usr/bin/env python3.8

import math
from pathlib import Path

import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, Image
import ros_numpy
from cv_bridge import CvBridge

import torch
import torch.utils.data as torch_data

import pandas as pd
import numpy as np
from skimage import io

from torch.utils.data._utils.collate import default_collate
from thop.profile import profile  # Import profile from thop

from pcdet.models import build_network, load_data_to_gpu
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.utils.calibration_kitti import Calibration

#Calib and config paths
CALIB_CAM_TO_CAM = "/workspace/lidar_config/calib_new.txt"
MODEL_CFG = "/workspace/lidar_config/VirConv-T-live.yaml"
MODEL_PATH = "/workspace/lidar_config/checkpoint_epoch_75.pth"

# PENet model configurations
NETWORK_MODEL = "pe"
DILATION_RATE = 2
INPUT_TYPE = "rgbd"
VAL_TYPE = "select"
CONVOLUTIONAL_ENCODING = "xyz"
USE_CPU = False

#ROS subcriber paths
POINTCLOUD_TOPIC = "/ouster/points"
CAMERA_IMAGE_TOPIC = "/camera/color/image_raw"

#Globals to ref the last point cloud and RGB image synced with an apprx sync
last_point_cloud: PointCloud2 = None
last_rgb_image: Image = None

# Load the model configs and save them into the cfg variable and use them to get the class names.
cfg_from_yaml_file(MODEL_CFG, cfg)
CLASS_NAMES = cfg.CLASS_NAMES

#Get a bridge to convert the camera's image to an NP array
bridge = CvBridge()

class LiveDataset(DatasetTemplate):
    """
    This class is designed to hopefully be a drop in replacement for the NewDataset class from last team, and
    create a bridge between datasets and live point cloud data. This will hopefully mean just presenting data
    as a batch of 1 inference per when loading and creating the model
    """

    def __init__(self, cfg):
        """
        Init is the constructor for our live streaming dataset
        """
        super().__init__(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            training=False,
            root_path=Path("/"), #Override to an empty path, we dont really have data that way
        )

        self.cfg = cfg
        self.frame_indx = 0
        self.calib_cache = Calibration(CALIB_CAM_TO_CAM)

    def __len__(self):
        """
        Unsure if this override is required, defaulted to inf so that this
        class inheritence doesnt die (I hope)
        """

        return math.inf
    
    def __getitem__(self, index):
        """
        An ovverride of getitem that stores that input dict that is fed through the model,
        this should fetch the most recent point cloud

        Returns:
        data_dict: Data dictonary that contains all the input data,calibration and image
            information needed for VirConv.
        """
        #Ref the global pointcloud and image, populated by the callback
        global last_rgb_image, last_point_cloud

        #Spin until data is ready
        spin_start = rospy.get_time()
        while ((last_rgb_image == None) and (last_point_cloud == None)):
            if (rospy.is_shutdown()):
                exit()
            if ((rospy.get_time() - spin_start) > 1):
                rospy.logwarn("Lidar node has been spinning for over 500ms!")
                spin_start = rospy.get_time()
            rospy.sleep(0.1)

        rospy.loginfo("Pointcloud and image data hit!")

        #Unpack the point cloud to NP array
        cloud_unpacked = ros_numpy.numpify(last_point_cloud)
        points = np.zeros((cloud_unpacked.shape[0] * cloud_unpacked.shape[1], 8), dtype=np.float32)
        points[:, 0] = cloud_unpacked["x"].ravel()
        points[:, 1] = cloud_unpacked["y"].ravel()
        points[:, 2] = cloud_unpacked["z"].ravel()
        points[:, 3] = cloud_unpacked["intensity"].ravel()

        points[:, 7] = 1.0 #Mask out to specify RGB channels?

        #Do the same with an image
        image = bridge.imgmsg_to_cv2(last_rgb_image)

        #Clean up
        last_rgb_image = None
        last_point_cloud = None
        
        calib = self.get_calib()
        #img_shape = self.get_image_shape(file_index)

        # Organize the data in the correct dictonary with the correct keys
        input_dict = {
            "points": points,
            "frame_id": self.frame_indx,
            "calib": calib
        }

        self.frame_indx += 1

        # Add the mm key to tell the prepare dataset what kind of data it's preparing
        # Prepare data does transformation of the points and adds dataset properties
        input_dict.update({"mm": np.ones(shape=(1, 1))})
        data_dict = self.prepare_data(data_dict=input_dict)

        # Add back the image shape and calibration files
        data_dict["image_shape"] = image.shape[:2]
        data_dict["calib"] = calib

        rospy.logwarn(f"After preparing data, the model has voxel coords: {data_dict["voxel_coords"].shape}")

        #Increment our useless frame ID
        return data_dict

    def get_calib(self):
        """Grabs and returns the calibraion information

        args: N/a

        Returns:
        Dictionary containig Transformation matricies fron the
            CALIB_CAM_TO_CAM and CALIB_VELO_TO_CAM calibration
            files.
        """

        return self.calib_cache

    def collate_batch(self, batch_list, _unused=False):
        """This is how the data that's fetched from this class is batched together according
        to our dataloader. Will return the input data dictonary as a batched set.

        args:
        :batch_list : List of individual samples that are meant to be batched together
            accoriding to our dataloader.

        Returns:
        data_dict: Dictonary that collated the individual samples to create the batched
            data.
        """

        data_dict = super().collate_batch(batch_list, _unused)
        return data_dict


def data_callback(rgb_data: Image, points: PointCloud2) -> None:
    """
    Syncing call back that will store the most recent image and point cloud,
    processing into numpy arrays happen at time of inference so that extra work
    is only done when needed
    """
    global last_point_cloud, last_rgb_image
    last_point_cloud = points
    last_rgb_image = rgb_data


class LidarTimeAdjuster(message_filters.SimpleFilter):
    def __init__(self):
        super(LidarTimeAdjuster, self).__init__()

        # First timestamps
        self.first_lidar_time = None
        self.first_camera_time = None
        self.offset_set = False
        self.offset = 0.0

        # Subscribe to LiDAR messages
        rospy.Subscriber(POINTCLOUD_TOPIC, PointCloud2, self.lidar_callback)

        # Subscribe to camera messages to get first timestamp
        self.camera_sub = rospy.Subscriber(CAMERA_IMAGE_TOPIC, Image, self.camera_callback)

    def camera_callback(self, camera_msg):
        if self.first_camera_time is None:
            self.first_camera_time = camera_msg.header.stamp.to_sec()

    def lidar_callback(self, lidar_msg):
        lidar_time = lidar_msg.header.stamp.to_sec()

        # Record first LiDAR timestamp
        if self.first_lidar_time is None:
            self.first_lidar_time = lidar_time

        # Calculate offset once both first messages are available
        if not self.offset_set and self.first_camera_time is not None:
            self.offset = self.first_camera_time - self.first_lidar_time
            self.offset_set = True

        # Apply offset if calculated
        if self.offset_set:
            lidar_msg.header.stamp = rospy.Time.from_sec(lidar_time + self.offset)

        # Emit the adjusted message to the synchronizer
        self.signalMessage(lidar_msg)

def main():
    # Initialize the ROS node and publisher
    rospy.init_node("lidar_node", anonymous=False)

    #Create the subscribers for the point cloud and rgb data
    rgb_sub = message_filters.Subscriber(CAMERA_IMAGE_TOPIC, Image)
    #points_sub = message_filters.Subscriber(POINTCLOUD_TOPIC, PointCloud2)
    points_sub = LidarTimeAdjuster()

    message_filters.ApproximateTimeSynchronizer(
        [rgb_sub, points_sub], 
        queue_size=40,
        slop=0.1
    ).registerCallback(data_callback)

    # Use the dataset instance to create the model
    dataset = LiveDataset(cfg)

    # Build the model using the YAML Configurations and dataset instance
    rospy.loginfo("Building the VirConv model architecture...")
    model = build_network(
        model_cfg=cfg.MODEL, num_class=len(CLASS_NAMES), dataset=dataset
    )

    #Get the model state
    state_dict = torch.load(MODEL_PATH, weights_only=False)
    model.load_state_dict(state_dict["model_state"])
    rospy.loginfo("Weights Loaded and Ready")

    # Move model to GPU and set to evaluation mode
    model.cuda()
    model.eval()

    rospy.loginfo("Model loaded and ready for inference.")


    #This cursed thing is designed so we have to pass it an index, the livedataset
    #override of the data set methods make this number largely useless but oh well
    current_frame = 0

    try:
        while not rospy.is_shutdown():
            rospy.loginfo("Inside main loop, checking shutdown status...")

            if rospy.is_shutdown():  # Double-check shutdown condition
                rospy.loginfo("ROS is shutting down, exiting...")
                break  # Exit the loop cleanly

            with torch.no_grad():
                rospy.loginfo("Inside inference loop...")
                
                if rospy.is_shutdown():
                    rospy.loginfo(
                        "ROS shutdown detected inside inference loop, exiting..."
                    )
                    break

                #Grab the next data
                raw_data = dataset[current_frame]
                prepared_data = dataset.collate_batch([raw_data])
                load_data_to_gpu(prepared_data)

                # Produces the object detection list
                pred_dicts, _, *rest = model.forward(prepared_data)

                rospy.loginfo(f"Model succsessfully forwarded, produced: {len(pred_dicts)}")

                current_frame += 1

            rospy.loginfo("Completed main loop cycle. Starting next cycle.")

    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught. Exiting...")
    finally:
        rospy.loginfo("Final cleanup before exit.")


if __name__ == "__main__":
    main()