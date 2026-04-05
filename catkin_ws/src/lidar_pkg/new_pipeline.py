#!/home/paccar/multimodal_perception/catkin_ws/src/virconv/src/virconv-master/virconv_env/bin/python3
# The line above tells ROS to use my version of Python,3.10, interpreter instead of the default Python 3.8

###############################################################################################################
# new_pipeline.py
# This is the pipeline responsible for loading raw input data from the LiDAR and Camera in KITTI format into
# the VirConv algorithm. This model is using the VirConv-T algorithm and produces the object list that are
# sent into the system using object_detection message. This will first run throught the PETNet algorithm that
# is responsible for preprocessing the camera and LiDAR data. The camera images will be converted into RGB
# virtual points and fused with the raw LiDAR points. This adds more features and information that one may lose
# to just using LiDAR data since it can be sparse. Then that output is fed into the VirConv algorithm.
#
# 6/4/2025 Sydney Dukelow
###############################################################################################################


import rospy
import os
import torch
import sys
import shutil

import pandas as pd
import numpy as np
from pathlib import Path
import torch.utils.data as torch_data
from skimage import io
from virconv.msg import ObjectDetection

from pcdet.models import build_network
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.utils.calibration_kitti import Calibration, get_calib_from_files_fallback

# from tools.PENet.dataloaders.kitti_loader import KittiDepth
from tools.PENet.model import ENet, PENet_C1, PENet_C2, PENet_C4
from tools.PENet import vis_utils

# from tools.PENet.dataloaders.my_loader import MyLoaderTwo
from tools.PENet.dataloaders.kitti_loader import KittiDepthTwo
from torch.utils.data._utils.collate import default_collate
from thop.profile import profile  # Import profile from thop

# General File path for Root directory of the data files.
DETPATH = "/mnt/c/Users/Junaid Khan/Desktop/Data/2011_09_26/2011_09_26_drive_0002_sync/"

# File paths for the raw sensor files and calibration files, plus the camera's path
CAM_DATA_DIR = "/mnt/c/Users/Junaid Khan/Desktop/Data/2011_09_26/2011_09_26_drive_0002_sync/image_02/data"
LIDAR_DATA_DIR = "/mnt/c/Users/Junaid Khan/Desktop/Data/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data"
CALIB_CAM_TO_CAM = (
    "/mnt/c/Users/Junaid Khan/Desktop/Data/2011_09_26/calib_cam_to_cam.txt"
)
CALIB_VELO_TO_CAM = (
    "/mnt/c/Users/Junaid Khan/Desktop/Data/2011_09_26/calib_velo_to_cam.txt"
)
CAM_DATA_PATH = Path(CAM_DATA_DIR)

# Velodyne depth location and path
FOLDER_TO_DELETE = "/mnt/c/Users/Junaid Khan/Desktop/Data/2011_09_26/2011_09_26_drive_0002_sync/velodyne_depth"
# VELODYNE_DEPTH_PATH = Path("/mnt/c/Users/Junaid Khan/Desktop/Data/2011_09_26/2011_09_26_drive_0002_sync/velodyne_depth")
VELODYNE_DEPTH_PATH = Path(FOLDER_TO_DELETE)

# Location of the VirConv Model configuration and checkpoint
MODEL_CFG = "/home/paccar/multimodal_perception/catkin_ws/src/virconv/src/virconv-master/tools/cfgs/models/kitti/VirConv-T.yaml"
MODEL_CHECKPOINT = "/home/paccar/multimodal_perception/catkin_ws/src/virconv/src/virconv-master/output/models/kitti/VirConv-T/default/ckpt/checkpoint_epoch_75.pth"

# Location for the PENet checkpoint path
CHECKPOINT_PATH = "/home/paccar/multimodal_perception/catkin_ws/src/virconv/src/virconv-master/tools/PENet/pe.pth"

# PENet model configurations
NETWORK_MODEL = "pe"
DILATION_RATE = 2
INPUT_TYPE = "rgbd"
VAL_TYPE = "select"
CONVOLUTIONAL_ENCODING = "xyz"
USE_CPU = False

# Directory for the current project and the PENet files
PROJECT_ROOT = (
    "/home/paccar/multimodal_perception/catkin_ws/src/virconv/src/virconv-master/"
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


# Load the model configs and save them into the cfg variable and use them to get the class names.
cfg_from_yaml_file(MODEL_CFG, cfg)
CLASS_NAMES = cfg.CLASS_NAMES


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
        output_root_path,
        val_h=352,
        val_w=1216,
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
        self.detpath = output_root_path


def custom_collate_fn(batch):
    """This collate function collates the batches so are produced from the
    KittiDepthTwo function. Returns data for only single-item batches.
    Additionally it will add back the frame index that corresponds to the
    actually sensor file number and the calibration information.

    args:
    batch : list of dictonaries with the data alongside its frame index
        and the calibration information.

    Returns:
    batched_data: Will batch the tensor data and add the calibration and frame ID
    """
    single_item_dict = batch[0]

    # Take out the calibration and frame ID to be added later
    calib_obj = single_item_dict.pop("calib_obj")
    frame_idx_str = single_item_dict.pop("frame_idx_str")

    batched_data = default_collate(batch)

    # Add back the non-tensor items
    batched_data["calib_obj"] = calib_obj
    batched_data["frame_idx_str"] = frame_idx_str

    return batched_data


def evaluate_and_get_velodyne_depth(
    checkpoint_path,
    output_root_path,
    camera_data_dir,
    lidar_data_dir,
    calib_cam_to_cam_path,
    calib_velo_to_cam_path,
    network_model="pe",
    dilation_rate=2,
    input_type="rgbd",
    val_type="select",
    convolutional_encoding="xyz",
    use_cpu=False,
):
    """Function that runs the raw sensor data into the PENet model. The PENet model
    will create virtual points from the camera frame and then fuse those virtual points
    with the LiDAR points. These will fuse to create the velodyne_depth files to contain
    both LiDAR and the virtual camera points.

    args:
    checkpoint_path : String that stores the data location for the PENet model
    output_root_path: String for the directory to store the output, .npy files containing
        the fused virtual RGB points and raw LiDAR points.
    camera_data_dir : String for the directory that stores the .png files
    lidar_data_dir : String for the directory that stores the .bin files
    calib_cam_to_cam_path : String for the directory that stores the CAM TO CAM calibration
        files.
    calib_velo_to_cam_path : String for the directory that stores the CAM to LIDAR
        calibration files.
    network_model : String that defines which network model is used, default pe for PETNet
    dilation_rate : Int/float that defines the dilation rate to be applied in convolutional
        layers
    input_type: String defining the type of camera data, default is RGB
    val_type: String that determines the validation type, default is the default type inside
        KittiDepthTwo.
    convolutional_encoding : String that defines the space that the model will predict the
        virtual points. Default is using 3D space to match the LiDAR points
    use_cpu: Bool that defines if the model is using the CPU rather than the GPU.

    Return:
    There is no return but it will produce .npy files that will contain the velodyne_depth
    information. It will be stored in ROOT directory/velodyne_depth file
    """
    # Determine the device (CUDA or CPU)
    cuda = torch.cuda.is_available() and not use_cpu
    device = torch.device("cuda" if cuda else "cpu")
    print(f"=> using '{device}' for computation.")

    if cuda:
        torch.backends.cudnn.benchmark = (
            True  # Enables cuDNN autotuner for faster performance
        )

    print(f"Loading model checkpoint from: {checkpoint_path}")

    checkpoint = torch.load(checkpoint_path, map_location=device)

    # Initialize DummyArgs for model and dataset configuration
    args = DummyArgs(
        network_model,
        dilation_rate,
        input_type,
        val_type,
        convolutional_encoding,
        use_cpu,
        output_root_path,
    )

    # Initialize the PENet model based on arguments
    model = None
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

    if model is None:
        raise ValueError(
            "PENet model could not be initialized. Check network_model and dilation_rate."
        )

    # Load the model's state dictionary from the checkpoint and set to inference mode
    model.load_state_dict(checkpoint["model"], strict=False)
    model.eval()

    # Instantiate KittiDepthTwo dataset that will grab the raw sensor data
    test_dataset = KittiDepthTwo(
        "test_completion",
        args,
        camera_data_dir,
        lidar_data_dir,
        calib_cam_to_cam_path,
        calib_velo_to_cam_path,
        output_root_path,
    )

    # Create DataLoader for batch processing from the test_dataset
    test_loader = torch_data.DataLoader(
        test_dataset,
        batch_size=1,
        shuffle=False,
        num_workers=5,
        pin_memory=True,
        collate_fn=custom_collate_fn,
    )
    total_iterations = len(test_loader)

    print("Generating velodyne_depth files...")

    # Create output directory if it doesn't exist at DETPAT_ROOT DIRECTORY/velodyne_depth
    output_dir = Path(output_root_path) / "velodyne_depth"
    output_dir.mkdir(parents=True, exist_ok=True)

    with torch.no_grad():
        for i, batch_data in enumerate(test_loader):
            # Move all tensor data to the GPU
            for key in batch_data:
                if torch.is_tensor(batch_data[key]):
                    batch_data[key] = batch_data[key].to(device)

            # Run the batched data through the PENet model
            pred = model(batch_data)
            pred_np = pred.squeeze().cpu().numpy()

            current_raw_image_np = batch_data["raw_image_np"]
            current_raw_lidar_np = batch_data["raw_lidar_np"]

            # Ensure image data is in the correct format (Height,Width,Center) if it's a tensor
            if torch.is_tensor(current_raw_image_np):
                current_raw_image_np = current_raw_image_np.squeeze(0).cpu().numpy()
                if current_raw_image_np.ndim == 3 and current_raw_image_np.shape[0] in [
                    1,
                    3,
                ]:  # C, H, W -> H, W, C
                    current_raw_image_np = current_raw_image_np.transpose(1, 2, 0)

            # Ensure LiDAR data is numpy if it's a tensor
            if torch.is_tensor(current_raw_lidar_np):
                current_raw_lidar_np = current_raw_lidar_np.squeeze(0).cpu().numpy()

            current_calib_obj = batch_data["calib_obj"]
            current_frame_idx_str = batch_data["frame_idx_str"]

            # Save the predicted depth as velodyne points and show counter while generation
            vis_utils.save_depth_as_points_two(
                pred_np,
                current_frame_idx_str,
                output_root_path,
                image_data=current_raw_image_np,
                lidar_data=current_raw_lidar_np,
                calib_obj=current_calib_obj,
            )
            print(f"\rProcessed: {i+1}/{total_iterations}", end="")

    print("\nVelodyne_depth generation complete.")


class NewDataset(DatasetTemplate):
    """This is the class responsible for loading the .npy files and the corroponding
    calibration files. This prepares the data to be run through VirConv
    model by grabbing the necessary information and the collate batch function

    args:
    DatasetTemplate : Base Dataset template containing base properties for loading
        data.

    Returns:
    data_dict: Dictionary containing the LiDAR, virtual RGB points, calibration
        files, individual frame IDs, and image size. This is formatted to VirConv's
        specifications for it's model input.
    """

    def __init__(self, data_dir, cfg):
        """This is the initalization function that will grab the initial data
        directory for the input streams and configurations. This will help
        contain the properties within the class.

        args:
        data_dir: String containing the root path for the sensor files,
        cfg: Variable containing all the configuration inputs for VirConv.

        Returns:
        N/A
        """
        super().__init__(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            training=False,
            root_path=Path(data_dir),
        )

        self.data_dir = Path(data_dir)
        self.file_list = self.include_all_files()
        print(f"Found {len(self.file_list)} .bin files.")
        self.cfg = cfg

    def get_lidar_mm(self, idx):
        """Grabs and returns the velodyne depth files, .npy.

        args:
        idx: Interger value containing which file is being read

        Returns:
        Loads the points contained in the npy file as a float 32.
        """

        lidar_file = VELODYNE_DEPTH_PATH / (f"{idx}.npy")
        assert lidar_file.exists()
        return np.load(lidar_file).astype(np.float32)

    def get_image_shape(self, idx):
        """Grabs and returns the Image shape from the raw .png file

        args:
        idx: Interger value containing which file is being read

        Returns:
        Loads the image and returns the shape as an integer 32.
        """

        img_file = CAM_DATA_PATH / (f"{idx}.png")
        assert img_file.exists()
        return np.array(io.imread(img_file).shape[:2], dtype=np.int32)

    def get_calib(self):
        """Grabs and returns the calibraion information

        args: N/a

        Returns:
        Dictionary containig Transformation matricies fron the
            CALIB_CAM_TO_CAM and CALIB_VELO_TO_CAM calibration
            files.
        """

        return Calibration(CALIB_CAM_TO_CAM, CALIB_VELO_TO_CAM)

    def include_all_files(self):
        """Reads the amount of LiDAR .bin files to define the amount of files and make sure
        no files are missing. Will return the amount of files.

        args:
        N/a

        Return:
        A list of sorted strings associated with the file names for each file such as
            [00000.bin,00001.bin,etc.].
        """

        velo_path = Path(LIDAR_DATA_DIR)
        all_files = []

        if velo_path.exists() and velo_path.is_dir():
            all_files = [f.stem for f in velo_path.iterdir() if f.suffix == ".bin"]
            all_files.sort()
        else:
            rospy.logerr(
                f"LIDAR data directory not found or is not a directory: {velo_path}. Cannot load .bin files."
            )

        return all_files

    def __len__(self):
        """Reads file list to know how many times to iteratively run through the class without
        missing any file.

        args:
        N/a

        Return:
        Integer containing the number of files.
        """

        return len(self.file_list)

    def __getitem__(self, index):
        """This will grab each indivdual file for each frame. So it will grab the
        corrosponding velodyne depth, image shape, calibrations files, and frame ids.

        args:
        index: The index value the class is grabbing associated with the .npy and .png files

        Returns:
        data_dict: Data dictonary that contains all the input data,calibration and image
            information needed for VirConv.
        """
        # Grab the associated .npy and image shapes with the current index
        # and calibration information
        file_index = self.file_list[index]
        points = self.get_lidar_mm(file_index)
        calib = self.get_calib()
        img_shape = self.get_image_shape(file_index)

        # Organize the data in the correct dictonary with the correct keys
        input_dict = {
            "points": points,
            "frame_id": index,
            "calib": calib,
        }

        # Add the mm key to tell the prepare dataset what kind of data it's preparing
        # Prepare data does transformation of the points and adds dataset properties
        input_dict.update({"mm": np.ones(shape=(1, 1))})
        data_dict = self.prepare_data(data_dict=input_dict)

        # Add back the image shape and calibration files
        data_dict["image_shape"] = img_shape
        data_dict["calib"] = calib

        return data_dict

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


def convert_batch_dict_to_tensors(batch_dict, device=None):
    """Converting the batched data for VirConv into the GPU

    args:
    batch_dict: Dictonary that contains the input data that is
        required by VirConv but in the CPU.

    Return:
    batch_dict: Dictonary that contains the input data but has
        moved it to the GPU.
    """
    for key, val in batch_dict.items():
        if not isinstance(val, torch.Tensor):
            try:
                batch_dict[key] = torch.tensor(val)
                if device is not None:
                    batch_dict[key] = batch_dict[key].to(device)
            except (TypeError, ValueError) as e:
                error = e
    return batch_dict


def manual_nms(boxes, scores, iou_threshold):
    """Performs Non-Maximum Suppression (NMS) manually but using the
    intersection over union.

    Returns:
    Will return only the objects abouve the iou threshold
    """

    # Extract coordinates for all bounding boxes
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    # Calculate the area of each bounding box and add 1 for inclusive coordinates
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)

    # Sort bounding boxes by their scores in descending order
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        # Get the index of the box with the highest score and compare boxes with this box
        i = order[0]

        keep.append(i)

        # Calculate the coordinates of the intersection area for the current box
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        # Calculate the width and height of the intersection area
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)

        # Calculate the area of the intersection
        intersection = w * h

        # Calculate the Intersection Over Union (IOU)
        iou = intersection / (areas[i] + areas[order[1:]] - intersection)

        # Finding the boxes whose IOU is less than or equal to the iou_threshold
        inds = np.where(iou <= iou_threshold)[0]

        # Reorders the remaining boxes
        order = order[inds + 1]

    return keep


def apply_manual_nms(df, iou_threshold=0.50):
    """Applies manual NMS to a DataFrame of bounding boxes.

    args:
    df: Dataframe that contains the raw object detections
    iou_threshold: Integer containg the max amount of IOU or
        overlap between bounding boxes.

    Return:
    A new dataframe with the remaining object detctions above
        the IOU threshold.
    """
    if df.empty:
        return df

    # Convert center-based (x, y, length, width) coordinates from the DataFrame
    # to (x1, y1, x2, y2) format and store into boxes
    boxes = np.zeros((len(df), 4))
    boxes[:, 0] = df["x"] - df["length"] / 2
    boxes[:, 1] = df["y"] - df["width"] / 2
    boxes[:, 2] = df["x"] + df["length"] / 2
    boxes[:, 3] = df["y"] + df["width"] / 2

    # Extract the confidence scores from the DataFrame as a NumPy array.
    scores = df["score"].values

    # Applies the nms function
    keep_indices = manual_nms(boxes, scores, iou_threshold)

    # filters the original DataFrame to include only the non-suppressed detections.
    return df.iloc[keep_indices]


def boxes3d_lidar_to_kitti_camera(boxes3d_lidar, calib):
    """Converts the output of the model, which is relative to the LiDAR, into coordinates relative to the camera.
    (x, y, z) is the box center and refrencing the center of the frame. To the left
    side is negative x, while right is positive. The y value is relative to the level
    that the camera is mounted. A negaive y is lower than where the camera is mounted,
    and positive is above.

    args:
    boxes3d_lidar: Dataframe that contains the object detections from the model, which are relative to
        the LiDAR.
    calib: Dictionary that contains the calibration information between the camera and LiDAR

    Returns:
    df: Dataframe that contains the processed object detections in relation to the KITTI Camera.
    """

    # Grabs the (x, y, z) center coordinates of the bounding boxes from the LiDAR data.
    xyz_lidar = boxes3d_lidar[:, 0:3]

    # Extract other properties of the bounding boxes
    l, w, h, r, s, label, class_id, frame = (
        boxes3d_lidar[:, 3:4],
        boxes3d_lidar[:, 4:5],
        boxes3d_lidar[:, 5:6],
        boxes3d_lidar[:, 6:7],
        boxes3d_lidar[:, 7:8],
        boxes3d_lidar[:, 8:9],
        boxes3d_lidar[:, 9:10],
        boxes3d_lidar[:, 10:11],
    )

    # Adjust the Z coordinate to the bottom center of the box
    # to conform with KITTI conventions
    xyz_lidar[:, 2] -= h.reshape(-1) / 2

    # Grabs calibration information between the LiDAR and Camera
    Tr_velo2cam = calib["Tr_velo2cam"]
    R0 = calib["R0"]

    # Adds another Column so it's a (x, y, z, 1) or 4x4 matrix
    pts_lidar_hom = np.hstack(
        (xyz_lidar, np.ones((xyz_lidar.shape[0], 1), dtype=np.float32))
    )

    # Perform the  transformation from LiDAR to Camera Rectified.
    pts_rect = np.dot(pts_lidar_hom, np.dot(Tr_velo2cam.T, R0.T))

    # Adjust the rotation angle to match camera's
    r = -r - np.pi / 2

    # Create new dataframe of of the new bounding box data in camera's coordinates
    df = pd.DataFrame(
        {
            "x": pts_rect[:, 0],
            "y": pts_rect[:, 1],
            "z": pts_rect[:, 2],
            "length": l.reshape(-1),
            "width": w.reshape(-1),
            "height": h.reshape(-1),
            "rotation": r.reshape(-1),
            "score": s.reshape(-1),
            "label": label.reshape(-1),
            "class_name": class_id.reshape(-1),
            "frame_id": frame.reshape(-1).astype(int),
        }
    )
    print(df)
    return df


def main():
    # Initialize the ROS node and publisher
    rospy.init_node("virconv_node", anonymous=False)
    rospy.loginfo("virconv booted successfully")
    pub = rospy.Publisher("/object_detections", ObjectDetection, queue_size=10)

    # Load calibration files
    calib = get_calib_from_files_fallback(CALIB_CAM_TO_CAM, CALIB_VELO_TO_CAM)

    # Use the dataset instance to create the model
    dataset = NewDataset(DETPATH, cfg)

    # Build the model using the YAML Configurations and dataset instance
    rospy.loginfo("Building the VirConv model architecture...")
    model = build_network(
        model_cfg=cfg.MODEL, num_class=len(CLASS_NAMES), dataset=dataset
    )

    # Load the model checkpoint
    rospy.loginfo(f"Loading model checkpoint from: {MODEL_CHECKPOINT}")
    checkpoint = torch.load(MODEL_CHECKPOINT, map_location="cuda")
    model.load_state_dict(checkpoint["model_state"])

    # Move model to GPU and set to evaluation mode
    model.cuda()
    model.eval()

    # #####################################################################################
    # # Uncommment for the FLOP calculations
    # rospy.loginfo("Preparing dummy input for FLOPs calculation...")

    # # Temporary dataloader here just to grab the batch size and same dataset instance
    # temp_dataloader = torch_data.DataLoader(
    #     dataset,
    #     batch_size=5,
    #     shuffle=False,
    #     collate_fn=dataset.collate_batch,
    #     num_workers=0,
    # )

    # try:
    #     # Create a dummy batch directory to be fed into the model
    #     dummy_batch_dict = next(iter(temp_dataloader))

    #     # Grabs the device, preferably the GPU, and move the dummy batch_dict into the GPU.
    #     dummy_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    #     dummy_converted_batch = convert_batch_dict_to_tensors(dummy_batch_dict, dummy_device)

    #     rospy.loginfo("Calculating model FLOPs and parameters...")

    #     # Calculates the MACs (Multiply-Accumulate operations) and parameters of the model using our dummy input
    #     macs, params = profile(model, inputs=(dummy_converted_batch,), verbose=False)

    #     # Multiply MACs by 2 to get FLOPs (Multiply-Accumulate operations are usually 2 FLOPs: 1 multiplication + 1 addition)
    #     flops = macs * 2

    #     rospy.loginfo(f"Model FLOPs: {flops / 1e9:.2f} GFLOPs") # Report in GFLOPs
    #     rospy.loginfo(f"Model Parameters: {params / 1e6:.2f} M") # Report in Millions of parameters

    # except Exception as e:
    #     rospy.logerr(f"Failed to calculate FLOPs: {e}")

    # ########################################################################################################

    rospy.loginfo("Model loaded and ready for inference.")

    start_time = rospy.Time.now()
    try:
        while not rospy.is_shutdown():
            rospy.loginfo("Inside main loop, checking shutdown status...")

            if rospy.is_shutdown():  # Double-check shutdown condition
                rospy.loginfo("ROS is shutting down, exiting...")
                break  # Exit the loop cleanly

            # Generate the Velodyne Depth files from the raw data
            predicted_depths = evaluate_and_get_velodyne_depth(
                checkpoint_path=CHECKPOINT_PATH,
                output_root_path=DETPATH,
                camera_data_dir=CAM_DATA_DIR,
                lidar_data_dir=LIDAR_DATA_DIR,
                calib_cam_to_cam_path=CALIB_CAM_TO_CAM,
                calib_velo_to_cam_path=CALIB_VELO_TO_CAM,
                network_model=NETWORK_MODEL,
                dilation_rate=DILATION_RATE,
                input_type=INPUT_TYPE,
                val_type=VAL_TYPE,
                convolutional_encoding=CONVOLUTIONAL_ENCODING,
                use_cpu=USE_CPU,
            )

            rospy.loginfo("Creating new dataloader for the generated depth files...")
            # A new instance of the NewDataset is created since the npy files are created
            # and this will prepare them to be loaded into the model.
            current_dataset = NewDataset(DETPATH, cfg)

            # Creates a dataloader that batches the data into 5
            dataloader = torch_data.DataLoader(
                current_dataset,
                batch_size=5,
                shuffle=False,
                collate_fn=current_dataset.collate_batch,
                num_workers=5,
            )

            frame_id = 0
            with torch.no_grad():
                rospy.loginfo("Inside inference loop...")
                for batch_dict in dataloader:
                    if rospy.is_shutdown():
                        rospy.loginfo(
                            "ROS shutdown detected inside inference loop, exiting..."
                        )
                        break

                    # Determines if the device is on the GPU or CPU
                    device = torch.device(
                        "cuda" if torch.cuda.is_available() else "cpu"
                    )

                    # Grabs the batch data and moves it to the GPU
                    converted_batch = convert_batch_dict_to_tensors(batch_dict, device)

                    # Produces the object detection list
                    pred_dicts, _, *rest = model.forward(converted_batch)

                    all_boxes_data_frames = []

                    # Iterate through each frame's predictions within the current batch.
                    for batch_idx, data_dict_single_frame in enumerate(pred_dicts):
                        current_frame_id = frame_id

                        # Determines if the model detected any objects by the presence of pred_boxes
                        if "pred_boxes" in data_dict_single_frame:

                            # Extract predictions:
                            boxes = data_dict_single_frame["pred_boxes"].cpu().numpy()
                            scores = data_dict_single_frame["pred_scores"].cpu().numpy()
                            labels = data_dict_single_frame["pred_labels"].cpu().numpy()

                            # Create a Pandas DataFrame for the detections of the current single frame.
                            df_single_frame = pd.DataFrame(
                                {
                                    "x": boxes[:, 0],
                                    "y": boxes[:, 1],
                                    "z": boxes[:, 2],
                                    "length": boxes[:, 3],
                                    "width": boxes[:, 4],
                                    "height": boxes[:, 5],
                                    "rotation": boxes[:, 6],
                                    "score": scores,
                                    "label": labels,
                                }
                            )

                            # Map the class number to the actual class name and add the frame id
                            label_to_class = {
                                i + 1: name for i, name in enumerate(cfg.CLASS_NAMES)
                            }
                            df_single_frame["class_name"] = df_single_frame[
                                "label"
                            ].map(label_to_class)
                            df_single_frame["frame_id"] = current_frame_id

                            # Add single-frame DataFrame to the list.
                            all_boxes_data_frames.append(df_single_frame)
                        else:
                            rospy.logwarn(
                                f"pred_boxes not found in pred_dicts for frame index: {current_frame_id} (batch index: {batch_idx})"
                            )

                    # After running through all the frames and grabbing the object detections in the batch,
                    # post-process the results.
                    if all_boxes_data_frames:
                        # Concatenate all single-frame DataFrames into one large DataFrame for the entire batch.
                        df_all_frames_in_batch = pd.concat(
                            all_boxes_data_frames, ignore_index=True
                        )

                        rospy.loginfo(
                            f"Concatenated {len(all_boxes_data_frames)} frames with {len(df_all_frames_in_batch)} total detections in this batch."
                        )

                        # Apply nms to remove redundant overlapping detections
                        df_nms = apply_manual_nms(df_all_frames_in_batch)

                        # Transform the object detection list to be in relation to the camera
                        # coordinates. Only send these specific columns.
                        New_data = boxes3d_lidar_to_kitti_camera(
                            df_nms[
                                [
                                    "x",
                                    "y",
                                    "z",
                                    "length",
                                    "width",
                                    "height",
                                    "rotation",
                                    "score",
                                    "label",
                                    "class_name",
                                    "frame_id",
                                ]
                            ].values,
                            calib,
                        )

                        # Iterate through each detected object in the transformed DataFrame.
                        for _, row in New_data.iterrows():
                            # Create a new ROS `ObjectDetection` message with the given information and publish.
                            msg = ObjectDetection()
                            msg.x = row["x"]
                            msg.y = row["y"]
                            msg.z = row["z"]
                            msg.length = row["length"]
                            msg.width = row["width"]
                            msg.height = row["height"]
                            msg.rotation = row["rotation"]
                            msg.score = row["score"]
                            msg.label = int(row["label"])
                            msg.class_name = str(row["class_name"])
                            msg.frame_id = int(row["frame_id"])

                            pub.publish(msg)
                            rospy.loginfo(
                                f"Published ObjectDetection for frame {msg.frame_id}:\n{msg}"
                            )
                    else:
                        rospy.logwarn(
                            "No predictions found for any frame in the current batch."
                        )

                    # Increment the frame id, as it records the objects detected in each batch of data
                    frame_id += 1

            # Delete the Velodyne Depth points to save disk space.
            if os.path.exists(FOLDER_TO_DELETE) and os.path.isdir(FOLDER_TO_DELETE):
                try:
                    shutil.rmtree(FOLDER_TO_DELETE)
                    rospy.loginfo(f"Deleted folder: {FOLDER_TO_DELETE}")
                except OSError as e:
                    rospy.logerr(f"Error deleting folder {FOLDER_TO_DELETE}: {e}")
            else:
                rospy.loginfo(
                    f"Folder not found or not a directory: {FOLDER_TO_DELETE}"
                )

            # Calculates and shows the total processing time of the batch.
            end_time = rospy.Time.now()
            processing_time = (end_time - start_time).to_sec()
            rospy.loginfo(
                f"Total processing time for this cycle: {processing_time:.3f} seconds"
            )
            start_time = rospy.Time.now()  # Reset timer for the next cycle

            rospy.loginfo("Completed main loop cycle. Starting next cycle.")

    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught. Exiting...")
    finally:
        rospy.loginfo("Final cleanup before exit.")


if __name__ == "__main__":
    main()
