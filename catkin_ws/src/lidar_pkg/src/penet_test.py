#!/usr/bin/env python3.8
import math
from pathlib import Path
import os
import sys
import rospy
import message_filters
from message_filters import SimpleFilter
from copy import deepcopy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

import torch
import numpy as np
from skimage import io
from coop_per_msgs.msg import ObjectDetection

from tools.PENet.model import ENet, PENet_C1, PENet_C2, PENet_C4
from tools.PENet.vis_utils import generate_depth_as_points_two
from tools.PENet.dataloaders.kitti_loader import KittiDepthTwo

from pcdet.models import build_network
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.utils.calibration_kitti import Calibration

# Calibration and model configuration paths
CALIB_VELO_TO_CAM = "/workspace/lidar_config/calib_velo_to_cam.txt"
CALIB_CAM_TO_CAM = "/workspace/lidar_config/calib_cam_to_cam.txt"
MODEL_CFG = "/workspace/lidar_config/VirConv-T.yaml"
MODEL_PATH = "/workspace/lidar_config/checkpoint_epoch_75.pth"

# PENet model configurations
CHECKPOINT_PATH = "/workspace/lidar_config/pe.pth"
NETWORK_MODEL = "pe"
DILATION_RATE = 2
INPUT_TYPE = "rgbd"
VAL_TYPE = "select"
CONVOLUTIONAL_ENCODING = "xyz"
USE_CPU = False

# ROS subscriber topics
POINTCLOUD_TOPIC = "/ouster/points"
CAMERA_IMAGE_TOPIC = "/camera/color/image_raw"

# Global variables to store the latest point cloud and RGB image
last_point_cloud: PointCloud2 = None
last_rgb_image: Image = None

# Directory for the current project and the PENet files
PROJECT_ROOT = (
    "/opt/virconv/VirConv"
)
PENET_TOOLS_DIR = os.path.join(PROJECT_ROOT, "tools", "PENet")

# Add project root and PENet tools directory to sys.path for module imports
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
    print(f"Added '{PROJECT_ROOT}' to sys.path for module imports.")

if PENET_TOOLS_DIR not in sys.path:
    sys.path.insert(0, PENET_TOOLS_DIR)
    print(
        f"Added '{PENET_TOOLS_DIR}' to sys.path for direct module access during checkpoint load."
    )

cfg_from_yaml_file(MODEL_CFG, cfg)
CLASS_NAMES = cfg.CLASS_NAMES

bridge = CvBridge()

class ClockAdjustedSource(SimpleFilter):
    """
    Emits incoming messages with header timestamps adjusted
    to a node-relative clock.
    """
    def __init__(self, topic, msg_type):
        super(ClockAdjustedSource, self).__init__()
        self.topic = topic
        self.msg_type = msg_type
        self.start_time = rospy.Time.now()

        self.sub = rospy.Subscriber(self.topic, self.msg_type, self.callback)

    def callback(self, msg):
        adjusted_msg = deepcopy(msg)
        elapsed = rospy.Time.now() - self.start_time
        adjusted_msg.header.stamp = self.start_time + elapsed
        self.signalMessage(adjusted_msg)

class DummyArgs:
    """Dummy Args are the properties required to build the PENet model. These are dummpy arguments
    because they will change when the model is populated. This class is primarily used to bundle
    configuration parameters into a single, easily accessible object.

    args:
    network_model : string pointing to the model file
    dilation_rate : int/float rate applied inside the convolutional layers
    input_type : string indicating the input data the model expects.
        A string indicating the types of input data the model expects. Determines
        which modalities (e.g., RGB, Depth, Gradients)  are enabled. Examples: "rgbd",
        "d", "g".
    val_type : string defining validation process.
    convolutional_encoding : bool indicating to use a specific encoding scheme.
    use_cpu : bool indicating if model is run on the CPU, otherwise GPU.
    output_root_path : string for where output files are to be stored.

    Return:
    Will return the properties stored witin the clsas as an accessible object.
    """

    def __init__(
        self,
        network_model,
        dilation_rate,
        input_type,
        val_type,
        convolutional_encoding,
        use_cpu,
        val_h=640,
        val_w=1280,
    ):
        self.network_model = network_model
        self.dilation_rate = dilation_rate
        self.input = input_type
        self.val = val_type
        self.convolutional_layer_encoding = convolutional_encoding
        self.use_rgb = "rgb" in input_type
        self.use_d = "d" in input_type
        self.use_g = "g" in input_type
        self.val_h = val_h
        self.val_w = val_w
        self.cpu = use_cpu

class LidarNode:
    def __init__(self):
        rospy.init_node("lidar_node", anonymous=False)

        # Load Penet model for depth prediction
        self.model, self.device = load_penet_model(
            CHECKPOINT_PATH,
            NETWORK_MODEL,
            DILATION_RATE,
            INPUT_TYPE,
            VAL_TYPE,
            CONVOLUTIONAL_ENCODING,
            USE_CPU
        )

        # Load the dataset
        self.dataset = LiveDataset(cfg)

        # Load VirConv model for object detection
        self.model2 = load_virconv_model(
            cfg,
            MODEL_PATH,
            self.dataset,
            False
        )

        # Setup message filters and synchronization
        rgb_sub = ClockAdjustedSource(CAMERA_IMAGE_TOPIC, Image)
        points_sub = ClockAdjustedSource(POINTCLOUD_TOPIC, PointCloud2)

        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, points_sub], queue_size=5, slop=0.01
        )
        sync.registerCallback(self.data_callback)

        self.last_rgb_image = None
        self.last_point_cloud = None

    def data_callback(self, rgb_data: Image, points: PointCloud2):
        try:
            # Convert ROS image
            cv_image = bridge.imgmsg_to_cv2(rgb_data, desired_encoding="bgr8")
            rgb_np = np.array(cv_image, dtype=np.uint8)

            # Save latest RGB
            self.last_rgb_image = rgb_np

        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")
            self.last_rgb_image = None
            return

        try:
            # Convert ROS PointCloud2 to numpy
            points_list = list(pc2.read_points(
                points,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True
            ))

            if len(points_list) == 0:
                lidar_np = np.zeros((0, 4), dtype=np.float32)
            else:
                lidar_np = np.array(points_list, dtype=np.float32).reshape(-1, 4)
            
            self.last_point_cloud = lidar_np

        except Exception as e:
            rospy.logerr(f"PointCloud conversion failed: {e}")
            self.last_point_cloud = np.zeros((0, 4), dtype=np.float32)
            return

        # Only run inference if both RGB image and point cloud are available
        if self.last_rgb_image is not None and self.last_point_cloud is not None:
            batch_dict = produce_velodyne(
                self.model,
                self.device,
                self.last_rgb_image,
                self.last_point_cloud,
                CALIB_CAM_TO_CAM,
                CALIB_VELO_TO_CAM,
                self.dataset
            )
            
            batch_dict['image_shape'] = cv_image.shape[:2]

            # Strip points down to last 8 features
            # batch_dict["points"] = batch_dict["points"][:, 1:]

            with torch.no_grad():
                pred_dicts, _, *rest = self.model2.forward(batch_dict)
                rospy.loginfo(pred_dicts)

                most_confidence = pred_dicts[0]["pred_scores"].cpu().max()
                rospy.loginfo(f"Max Conf: {most_confidence}")
        else:
            rospy.logwarn("Either RGB image or PointCloud data is missing. Skipping inference.")

class LiveDataset(DatasetTemplate):
    """Dataset class for live streaming data using last_rgb_image and last_point_cloud."""

    def __init__(self, cfg):
        """Initialize the dataset with configuration."""
        super().__init__(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            training=False,
            root_path=Path("/tmp"),  # no data directory needed
        )
        self.cfg = cfg
        self.frame_counter = 0  # keep track of frames

    def get_lidar_mm(self):
        """Return the latest LiDAR points from the global variable."""
        global last_point_cloud
        if last_point_cloud is None:
            return np.zeros((0, 4), dtype=np.float32)
        return last_point_cloud.astype(np.float32)

    def get_image_shape(self):
        """Return the latest image shape from the global variable."""
        global last_rgb_image
        if last_rgb_image is None:
            return np.array([0, 0], dtype=np.int32)
        return np.array(last_rgb_image.shape[:2], dtype=np.int32)

    def get_image(self):
        """Return the latest RGB image."""
        global last_rgb_image
        return last_rgb_image

    def get_calib(self):
        
        # Pass calib_data as a single argument if the Calibration class expects one argument
        return Calibration(CALIB_CAM_TO_CAM, CALIB_VELO_TO_CAM)

    def __len__(self):
        """For streaming data, length can be large or undefined."""
        return 2**31  # effectively infinite

    def __getitem__(self, index):
        """Fetch the latest live data frame."""
        points = self.get_lidar_mm()
        img_shape = self.get_image_shape()
        calib = self.get_calib()

        input_dict = {
            "points": points,
            "frame_id": self.frame_counter,
            "calib": calib,
        }

        # Add mm key for VirConv
        input_dict.update({"mm": np.ones((1, 1))})
        data_dict = self.prepare_data(data_dict=input_dict)

        # Add image shape and calibration
        data_dict["image_shape"] = img_shape
        data_dict["calib"] = calib

        self.frame_counter += 1
        return data_dict

    def collate_batch(self, batch_list, _unused=False):
        """Batching remains the same as base DatasetTemplate."""
        data_dict = super().collate_batch(batch_list, _unused)
        return data_dict

def load_penet_model(
    checkpoint_path,
    network_model,
    dilation_rate,
    input_type,
    val_type,
    convolutional_encoding,
    use_cpu,
):
    cuda = torch.cuda.is_available() and not use_cpu
    device = torch.device("cuda")

    if cuda:
        torch.backends.cudnn.benchmark = True

    print(f"Loading model checkpoint from: {checkpoint_path}")
    checkpoint = torch.load(checkpoint_path, map_location=device)

    global args
    args = DummyArgs(
        network_model,
        dilation_rate,
        input_type,
        val_type,
        convolutional_encoding,
        use_cpu,
    )

    # Initialize model
    if args.network_model == "e":
        model = ENet(args).to(device)
    else:
        if args.dilation_rate == 1:
            model = PENet_C1(args).to(device)
        elif args.dilation_rate == 2:
            model = PENet_C2(args).to(device)
        elif args.dilation_rate == 4:
            model = PENet_C4(args).to(device)
        else:
            raise ValueError(f"Unsupported dilation rate: {args.dilation_rate}")

    model.load_state_dict(checkpoint["model"], strict=False)
    model.eval()

    return model, device

def load_virconv_model(cfg, model_path, dataset, use_cpu=False):
    """
    Load a VirConv model with specified configuration and dataset.

    Args:
        cfg: Configuration object with model settings (e.g., cfg.MODEL)
        model_path: Path to saved model weights
        dataset: Dataset instance (needed for number of classes, etc.)
        use_cpu: If True, forces CPU usage even if CUDA is available

    Returns:
        model: Loaded VirConv model in evaluation mode
        device: torch.device used for computation
    """
    # Determine device
    cuda = torch.cuda.is_available() and not use_cpu
    device = torch.device("cuda")
    rospy.loginfo(f"Using device: {device}")

    # Build the model using config and dataset info
    rospy.loginfo("Building the VirConv model architecture...")
    model = build_network(
        model_cfg=cfg.MODEL,
        num_class=len(dataset.class_names),
        dataset=dataset
    )

    # Load model state dict
    rospy.loginfo(f"Loading model weights from: {model_path}")
    state_dict = torch.load(model_path, map_location=device)
    model.load_state_dict(state_dict["model_state"])
    
    # Move model to device and set to eval mode
    model.cuda()
    model.eval()
    
    rospy.loginfo("VirConv model loaded and ready for inference.")
    
    return model

def produce_velodyne(model, device, last_rgb_image, last_point_cloud, calib_cam_to_cam_path, calib_velo_to_cam_path, dataset):
    """
    Produces a pseudo-LiDAR batch_dict from the latest RGB image and LiDAR points,
    ready for VirConv inference.
    """
    test_dataset = KittiDepthTwo(
        "test_completion",
        args,
        last_rgb_image,
        last_point_cloud,
        calib_cam_to_cam_path,
        calib_velo_to_cam_path
    )
    sample = test_dataset[0]

    for k, v in sample.items():
        if torch.is_tensor(v):
            sample[k] = v.unsqueeze(0).to(device)

    with torch.no_grad():
        pred = model(sample)

    pred_np = pred.squeeze().cpu().numpy()

    current_raw_image_np = last_rgb_image
    current_raw_lidar_np = last_point_cloud
    current_calib_obj = sample["calib_obj"]

    velodyne_points = generate_depth_as_points_two(
        pred_np,
        None,
        None,
        image_data=current_raw_image_np,
        lidar_data=current_raw_lidar_np,
        calib_obj=current_calib_obj,
    )

    batch_dict = {
        "points": velodyne_points.astype(np.float32),
        "frame_id": 0,
        "calib": current_calib_obj,
        "mm": np.ones((1, 1), dtype=np.float32),
    }

    batch_dict = dataset.prepare_data(batch_dict)

    calib_obj = Calibration(CALIB_CAM_TO_CAM, CALIB_VELO_TO_CAM)
    batch_calib = [calib_obj]
    batch_dict['calib'] = batch_calib

    for k, v in batch_dict.items():
        if isinstance(v, np.ndarray):
            batch_dict[k] = torch.tensor(v, dtype=torch.float32, device=device)

    for key in batch_dict:
        if 'voxel_coords' in key:
            coords = batch_dict[key]

            if coords.shape[1] == 3:
                # reorder if needed
                coords = coords[:, [2, 1, 0]]

                batch_idx = torch.zeros((coords.shape[0], 1), dtype=coords.dtype, device=coords.device)
                coords = torch.cat([batch_idx, coords], dim=1)

                batch_dict[key] = coords

    if 'transform_param' in batch_dict:
        param = batch_dict['transform_param']
        if param.ndim == 2:  # shape [3, 3]
            batch_dict['transform_param'] = param.unsqueeze(0)  # shape [1, 3, 3]

    for key, tensor in batch_dict.items():
        if isinstance(tensor, torch.Tensor):
            print(key, tensor.shape)


    batch_dict['batch_size'] = 1

    return batch_dict

def main():
    LidarNode()
    rospy.spin()

if __name__ == "__main__":
    main()