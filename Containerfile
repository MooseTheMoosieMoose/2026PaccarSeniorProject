#--------------------------
#Base on NVIDIAS optimized pytorch image
#--------------------------
FROM nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

#===============================================================================================

#                                                            ,--.                               
# ,-----.                                                   /  /     ,------.  ,-----.  ,---.   
#'  .--./ ,---. ,--,--,--.,--,--,--. ,---. ,--,--,         /  /      |  .--. ''  .-.  ''   .-'  
#|  |    | .-. ||        ||        || .-. ||      \       /  /       |  '--'.'|  | |  |`.  `-.  
#'  '--'\' '-' '|  |  |  ||  |  |  |' '-' '|  ||  |      /  /        |  |\  \ '  '-'  '.-'    | 
# `-----' `---' `--`--`--'`--`--`--' `---' `--''--'     /  /         `--' '--' `-----' `-----'  
#                                                      `--'                                     

#===============================================================================================


#--------------------------
#Get all of our base tools
#Requirements are based on the requirements section here:
#   https://github.com/ouster-lidar/ouster-ros/tree/master
#--------------------------
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libpcap-dev \
    cmake \
    git \
    python-dev \
    udev \
    pkg-config \
    libglfw3-dev \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

#--------------------------
#Get some extra ROS tools, along with python libs
#--------------------------
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
        numpy \
        pandas \
        rosdep \
        catkin-tools \
        rosinstall \
        rosinstall-generator \
        matplotlib \
        pillow \
        tqdm \
        scipy \
        tensorboard \
        seaborn \
        wstool \
        xacro

#--------------------------
#Get ROS and init ROS deps
#--------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros1.list

RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-pcl-ros \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

#--------------------------
#Create a link so that all python usage goes through our install of 3
#--------------------------
RUN ln -sf /usr/bin/python3 /usr/bin/python

#===============================================================================================
                                                       
# ,-----.  ,---.  ,--.   ,--.,------.,------.   ,---.   
#'  .--./ /  O  \ |   `.'   ||  .---'|  .--. ' /  O  \  
#|  |    |  .-.  ||  |'.'|  ||  `--, |  '--'.'|  .-.  | 
#'  '--'\|  | |  ||  |   |  ||  `---.|  |\  \ |  | |  | 
# `-----'`--' `--'`--'   `--'`------'`--' '--'`--' `--' 
                                                       
#===============================================================================================

#--------------------------
#YOLO by default will try and pull its own version of OpenCV which is very naughty
# so we have to bully it and install our own dependencies ourself
#--------------------------
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install ultralytics --no-deps

#--------------------------
#Pull and build the Intel Realsense SDK
#--------------------------
RUN git clone https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && \
    git checkout v2.54.2 && \
    mkdir build && cd build && \
    cmake .. \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DBUILD_PYTHON_BINDINGS=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_TOOLS=OFF \
        -DFORCE_RSUSB_BACKEND=ON && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/librealsense

#--------------------------
#Pull down the camera ROS node
#--------------------------
RUN mkdir -p /opt/camera_ws/src && \
    git clone -b ros1-legacy https://github.com/realsenseai/realsense-ros.git /opt/camera_ws/src

#--------------------------
#Rosdep for the camera ROS node
#--------------------------
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /opt/camera_ws && rosdep install --from-paths src --ignore-src -r -y || true"

#--------------------------
#Rosdep unfortunatly does not work with directly pulling cv_bridge, so we have to build that from source
#--------------------------
RUN apt-get update && apt-get install -y \
      libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/cv_bridge_ws/src 

RUN git clone -b noetic https://github.com/ros-perception/vision_opencv.git \
    /opt/cv_bridge_ws/src/vision_opencv

#Focus in on the cv_bridge workspace
WORKDIR /opt/cv_bridge_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /opt/cv_bridge_ws && \
    catkin_make \
      -DCMAKE_BUILD_TYPE=Release \
      -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 && \
    catkin_make install"

#Try multiple things to force the CV bridge build to be seen on the outside
RUN echo 'source /opt/cv_bridge_ws/install/setup.bash' >> /etc/bash.bashrc
ENV CMAKE_PREFIX_PATH=/opt/cv_bridge_ws/install:$CMAKE_PREFIX_PATH
RUN echo 'source /opt/cv_bridge_ws/install/setup.bash' >> /etc/bash.bashrc

#Pull back out to install root
WORKDIR /

#--------------------------
#Actually build the camera driver node and source it so its accessible
#--------------------------
RUN apt-get update && apt-get install -y \
      ros-noetic-image-transport \
      ros-noetic-tf \
      ros-noetic-ddynamic-reconfigure \
      ros-noetic-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    source /opt/cv_bridge_ws/install/setup.bash && \
    cd /opt/camera_ws && \
    catkin_make"

RUN echo "source /opt/camera_ws/devel/setup.bash" >> /root/.bashrc

#===============================================================================================

#,--.   ,--.,------.    ,---.  ,------.  
#|  |   |  ||  .-.  \  /  O  \ |  .--. ' 
#|  |   |  ||  |  \  :|  .-.  ||  '--'.' 
#|  '--.|  ||  '--'  /|  | |  ||  |\  \  
#`-----'`--'`-------' `--' `--'`--' '--'                                        

#===============================================================================================

RUN apt-get update && apt-get install -y \
    libzip-dev \
    && rm -rf /var/lib/apt/lists/*

#--------------------------
#Fetch down the Ouster driver node and build it
#--------------------------
RUN mkdir -p /opt/lidar_ws/src
WORKDIR /opt/lidar_ws/src
RUN git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
WORKDIR /opt/lidar_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release"
WORKDIR /

#--------------------------
#Now we need to resolve the necessary dependancies for VirConv, which are listed, and hopefully
#mostly contained in the base image here: https://github.com/hailanyi/VirConv
#--------------------------
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
        numba \
        pccm

#Build CuMM so we can eventually build SPConv
ENV CUMM_CUDA_VERSION="11.4"
ENV CUMM_DISABLE_JIT="1"
ENV CUMM_CUDA_ARCH_LIST="8.7"
RUN mkdir -p /opt/cumm_lib
WORKDIR /opt/cumm_lib
RUN git clone --branch v0.4.5 https://github.com/FindDefinition/cumm.git
RUN cd /opt/cumm_lib/cumm && python3 setup.py bdist_wheel && pip install dist/*.whl
WORKDIR /

#Now we can build SPConv, this takes for freaking EVER
ENV MAX_JOBS=1
ENV CMAKE_BUILD_PARALLEL_LEVEL=1
ENV SPCONV_DISABLE_JIT="1"
RUN mkdir -p /opt/spconv_lib
WORKDIR /opt/spconv_lib
RUN git clone --branch v2.3.5 https://github.com/traveller59/spconv.git
RUN cd /opt/spconv_lib/spconv && python3 setup.py bdist_wheel && pip install dist/*.whl
WORKDIR /

#--------------------------
#Now we have all the deps for VirConv but now we need all the deps for the full Lidar node
#--------------------------
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
        scikit-image thop rosnumpy

#and get pcdet
ENV TORCH_CUDA_ARCH_LIST="8.7"
RUN mkdir -p /opt/pcdet_lib
WORKDIR /opt/pcdet_lib
RUN git clone https://github.com/open-mmlab/OpenPCDet.git
WORKDIR /opt/pcdet_lib/OpenPCDet
RUN pip install -e .
WORKDIR /

#--------------------------
#In order to use VirConv in our project, we have to perform a very annoying set of
#tasks:
#   1. it has to be cloned
#   2. the lines "#include <THC/THC.h>" and
#                "extern THCState *state;" must be removed
#   3. then wheel and install
#--------------------------
RUN mkdir -p /opt/virconv
WORKDIR /opt/virconv
RUN git clone https://github.com/hailanyi/VirConv.git

#Now that its pulled we have to search and remove our targets
RUN find /opt/virconv -type f -exec grep -l '#include <THC/THC.h>' {} \; -exec sed -i '/#include <THC\/THC.h>/d' {} \; \
    && find /opt/virconv -type f -exec grep -l 'extern THCState \*state;' {} \; -exec sed -i '/extern THCState \*state;/d' {} \;

#For some reason setup.py does not like the fact that we built spconv from source, so we have to remove it from
#Virconv's dependencies list. Unfortunatly, this means employing black magic I dont understand; dont ask me how it works
#just know that it removes the `setup.py` dependency from the build portion of VirConv
RUN sed -i '/spconv/d' /opt/virconv/VirConv/setup.py

#And now we can finally try and build a working version of virconv from source :)
ENV CUDA_HOME="/usr/local/cuda"
WORKDIR /opt/virconv/VirConv
RUN pip install -e .
WORKDIR /

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
        prefetch-generator \
        tensorboardX \
        easydict \
        pyyaml

#===============================================================================================
                                               
#,--.                                 ,--.      
#|  |    ,--,--.,--.,--.,--,--,  ,---.|  ,---.  
#|  |   ' ,-.  ||  ||  ||      \| .--'|  .-.  | 
#|  '--.\ '-'  |'  ''  '|  ||  |\ `--.|  | |  | 
#`-----' `--`--' `----' `--''--' `---'`--' `--' 

#===============================================================================================

#--------------------------
#Launch to shell for dev
#--------------------------
CMD ["/bin/bash"]