#!/bin/bash

# 1. Validate inputs
if [ "$#" -eq 0 ]; then
    echo "Error: Missing roslaunch arguments."
    echo "Usage: ./launch_new.sh <package_name> <launch_file_name> [args...]"
    exit 1
fi

# 2. Source the base ROS Noetic installation
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo "Sourced base ROS Noetic."
else
    echo "Error: Base ROS Noetic setup not found at /opt/ros/noetic/setup.bash."
    exit 1
fi

# 3. Helper function to check, build, and source a workspace
setup_workspace() {
    local ws_path=$1
    
    if [ ! -d "$ws_path" ]; then
        echo "Warning: Directory $ws_path does not exist. Skipping."
        return
    fi

    # If the setup file doesn't exist, assume the workspace needs to be built
    if [ ! -f "$ws_path/devel/setup.bash" ]; then
        echo "Setup file missing in $ws_path. Initiating build..."
        pushd "$ws_path" > /dev/null || exit
        
        # Note: Replace 'catkin_make' with 'catkin build' if you use catkin_tools
        catkin_make 
        
        popd > /dev/null || exit
    fi

    # Source the workspace if the build was successful (or already existed)
    if [ -f "$ws_path/devel/setup.bash" ]; then
        source "$ws_path/devel/setup.bash" --extend
        echo "Sourced overlay: $ws_path"
    else
        echo "Error: Failed to build or find $ws_path/devel/setup.bash."
        exit 1
    fi
}

# 4. Chain the workspaces in your required overlay order
setup_workspace "/opt/camera_ws"
setup_workspace "/opt/lidar_ws"
setup_workspace "/opt/cv_bridge_ws"
setup_workspace "/workspace/catkin_ws"

echo "----------------------------------------"
echo "Executing: roslaunch $@"
echo "ROS Package Path:"
echo $ROS_PACKAGE_PATH
echo "----------------------------------------"

# 5. Run the provided roslaunch command with all passed arguments
roslaunch "$@"