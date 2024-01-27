# Docker image for building the project
# Environment: Ubuntu 20.04, ROS Noetic, CUDA 11.8, CuDNN 8.6.0, TensorRT 8.5.3, OpenCV 4.8

# docker buildx build \
#     --load \
#     --platform $(uname)/$(uname -m) \
#     --build-arg UBUNTU_VERSION=$UBUNTU_VERSION \
#     --build-arg CUDA_VERSION=$CUDA_VERSION \
#     --build-arg CUDNN_VERSION=$CUDNN_VERSION \
#     --build-arg TENSORRT_VERSION=$TENSORRT_VERSION \
#     --build-arg INSTALL_AS_DEV=$INSTALL_AS_DEV \
#     --tag $IMAGE \
#     .

ARG UBUNTU_VERSION=20.04

# === base-amd64-20.04 ===============================================================
FROM --platform=amd64 ubuntu:20.04 as base-amd64-20.04

# === base-arm64-20.04 ===============================================================
FROM --platform=arm64 nvcr.io/nvidia/l4t-base:35.4.1 as base-arm64-20.04

# === base-amd64-22.04 ===============================================================
FROM --platform=amd64 ubuntu:22.04 as base-amd64-22.04

# === base-arm64-22.04 ===============================================================
# not supported

# === dependencies =============================================================
FROM "base-${TARGETARCH}-${UBUNTU_VERSION}" as dependencies
ARG TARGETARCH
ENV DEBIAN_FRONTEND=noninteractive

ARG UBUNTU_VERSION
ARG CUDA_VERSION=11.8
ARG CUDNN_VERSION=8.6.0.163-1
ARG TENSORRT_VERSION=8.5.3-1
ARG INSTALL_AS_DEV=1

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

SHELL ["/bin/bash", "-c"]

# install essentials
RUN apt-get update && \
    apt-get install -y \
        wget \
        gnupg2 && \
    if [[ "$INSTALL_AS_DEV" == "1" ]]; then apt-get install -y build-essential; fi && \
    rm -rf /var/lib/apt/lists/*

# set up CUDA apt repositories
RUN if [[ "$TARGETARCH" == "amd64" ]]; then \
        wget -q -O /tmp/cuda-keyring_1.0-1_all.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu${UBUNTU_VERSION/./}/x86_64/cuda-keyring_1.0-1_all.deb; \     
    elif [[ "$TARGETARCH" == "arm64" ]]; then \
        wget -q -O /tmp/cuda-keyring_1.0-1_all.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu${UBUNTU_VERSION/./}/arm64/cuda-keyring_1.0-1_all.deb; \
    fi && \
    dpkg -i /tmp/cuda-keyring_1.0-1_all.deb && \
    rm -rf /tmp/cuda-keyring_1.0-1_all.deb

# set up L4T apt repositories (NVIDIA Jetson)
RUN if [[ "$TARGETARCH" == "arm64" ]]; then \
        echo "deb https://repo.download.nvidia.com/jetson/common r35.4 main" >> /etc/apt/sources.list && \
        echo "deb https://repo.download.nvidia.com/jetson/t194 r35.4 main" >> /etc/apt/sources.list && \
        apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc && \
        mkdir -p /opt/nvidia/l4t-packages/ && \
        touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall && \
        rm -rf /var/lib/apt/lists/* ; \
    fi

# install CUDA
RUN apt-get update && \
    if [[ "$TARGETARCH" == "amd64" ]]; then \
        apt-get install -y cuda-libraries-${CUDA_VERSION/./-} cuda-toolkit-${CUDA_VERSION/./-} cuda-demo-suite-${CUDA_VERSION/./-}; \
        if [[ "$INSTALL_AS_DEV" == "1" ]]; then apt-get install -y cuda-toolkit-${CUDA_VERSION/./-} cuda-demo-suite-${CUDA_VERSION/./-}; fi \
    elif [[ "$TARGETARCH" == "arm64" ]]; then \
        echo "N" | apt-get install -y cuda-runtime-${CUDA_VERSION/./-}; \
        if [[ "$INSTALL_AS_DEV" == "1" ]]; then apt-get install -y cuda-toolkit-${CUDA_VERSION/./-}; fi \
    fi && \
    echo "export PATH=/usr/local/cuda/bin:$PATH" >> ~/.bashrc && \
    rm -rf /var/lib/apt/lists/*

# install cuDNN
RUN --mount=type=bind,source=cudnn-local-repo-${TARGETARCH}.deb,target=/cudnn-local-repo.deb,ro \
    gpg_key=$(dpkg -i cudnn-local-repo.deb | grep -oP "(?<=cp )[^ ]+(?= )") && \
    cp ${gpg_key} /usr/share/keyrings/ && \
    CUDNN_MAJOR_VERSION=${CUDNN_VERSION%%.*} && \
    apt-get update && \
    apt-get install -y libcudnn${CUDNN_MAJOR_VERSION}=8.6.0.163-1+cuda11.8 libcudnn${CUDNN_MAJOR_VERSION}-dev=8.6.0.163-1+cuda11.8 && \
    if [[ "$INSTALL_AS_DEV" == "1" ]]; then \
        apt-get install -y libcudnn${CUDNN_MAJOR_VERSION}-dev=8.6.0.163-1+cuda11.8; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# install TensorRT
RUN --mount=type=bind,source=nv-tensorrt-local-repo-${TARGETARCH}.deb,target=/nv-tensorrt-local-repo.deb,ro \
    gpg_key=$(dpkg -i nv-tensorrt-local-repo.deb | grep -oP "(?<=cp )[^ ]+(?= )") && \
    cp ${gpg_key} /usr/share/keyrings/ && \
    TENSORRT_MAJOR_VERSION=${TENSORRT_VERSION%%.*} && \
    apt-get update && \
    apt-get install -y \
        libnvinfer${TENSORRT_MAJOR_VERSION}=8.5.3-1+cuda11.8 \
        libnvinfer-plugin${TENSORRT_MAJOR_VERSION}=8.5.3-1+cuda11.8 libnvinfer-dev=8.5.3-1+cuda11.8 libnvinfer-plugin-dev=8.5.3-1+cuda11.8 && \
    if [[ "$INSTALL_AS_DEV" == "1" ]]; then \
        apt-get install -y \
        libnvinfer-dev=${TENSORRT_VERSION}+cuda${CUDA_VERSION} \
        libnvinfer-plugin-dev=${TENSORRT_VERSION}+cuda${CUDA_VERSION}; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# container startup setup
# CMD ["bash"]

# Install basic packages
RUN apt-get update
RUN apt-get install -y wget vim git
RUN apt install nlohmann-json3-dev 
 
# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# ==================================================================
# Install OpenCV 
ARG OPENCV_VERSION="4.8.0"
RUN apt-get clean && \
    apt-get update && \
    apt-get install -y --no-install-recommends --fix-missing \
        build-essential binutils \
        ca-certificates cmake cmake-qt-gui curl \
        dbus-x11 \
        ffmpeg \
        gdb gcc g++ gfortran git \
        tar \
        lsb-release \
        procps \
        manpages-dev \
        unzip \
        zip \
        wget \
        xauth \
        swig \
        python3-pip python3-dev python3-numpy python3-distutils \
        python3-setuptools python3-pyqt5 python3-opencv \
        libboost-python-dev libboost-thread-dev libatlas-base-dev libavcodec-dev \
        libavformat-dev libavutil-dev libcanberra-gtk3-module libeigen3-dev \
        libglew-dev libgl1-mesa-dev libgl1-mesa-glx libglib2.0-0 libgtk2.0-dev \
        libgtk-3-dev libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev liblapack-dev \
        liblapacke-dev libopenblas-dev libopencv-dev libpng-dev libpostproc-dev \
        libpq-dev libsm6 libswscale-dev libtbb-dev libtbb2 libtesseract-dev \
        libtiff-dev libtiff5-dev libv4l-dev libx11-dev libxext6 libxine2-dev \
        libxrender-dev libxvidcore-dev libx264-dev libgtkglext1 libgtkglext1-dev \
        libgstreamer-plugins-base1.0-dev libglx-dev \ 
        libgstreamer1.0-dev libopenexr-dev libqt5svg5-dev qt5-default \
        openexr \
        pkg-config \
        qv4l2 \
        v4l-utils \
        zlib1g-dev \
        locales \
        && locale-gen en_US.UTF-8 \
        && LC_ALL=en_US.UTF-8 \
        && rm -rf /var/lib/apt/lists/* \
        && apt-get clean

WORKDIR /opencv
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
    && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip \
    && unzip opencv.zip \
    && unzip opencv_contrib.zip \
    && mv opencv-${OPENCV_VERSION} opencv \
    && mv opencv_contrib-${OPENCV_VERSION} opencv_contrib

RUN mkdir /opencv/opencv/build
WORKDIR /opencv/opencv/build

RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/opencv/opencv_contrib/modules ..

RUN make -j$(nproc)
RUN make install
RUN ldconfig
 
# ==================================================================
# Install ROS Noetic
WORKDIR /
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN apt update && \
    apt install -y \
        python3-pip \
        python3-tk \
        ros-noetic-desktop-full \
        build-essential \
        ros-noetic-joy \
        ros-noetic-tf2-sensor-msgs \
        ros-noetic-rosbash \
        ros-noetic-rviz \
        ros-noetic-teleop-twist-joy \
        ros-noetic-teleop-twist-keyboard \
        ros-noetic-laser-proc \
        ros-noetic-rgbd-launch \
        ros-noetic-depthimage-to-laserscan \
        ros-noetic-rosserial-arduino \
        ros-noetic-rosserial-python \
        ros-noetic-rosserial-server \
        ros-noetic-rosserial-client \
        ros-noetic-rosserial-msgs \
        ros-noetic-amcl \
        ros-noetic-map-server \
        ros-noetic-move-base \
        ros-noetic-urdf \
        ros-noetic-robot-state-publisher \
        ros-noetic-xacro \
        ros-noetic-compressed-image-transport \
        ros-noetic-rqt-image-view \
        ros-noetic-gmapping \
        ros-noetic-navigation \
        ros-noetic-interactive-markers

# ==================================================================
# Setup GUI dependencies

WORKDIR /${USER}

RUN git clone git://code.qt.io/qt/qtmqtt.git
WORKDIR /${USER}/qtmqtt
RUN git checkout v5.12.5
RUN apt-get -y install qtbase5-private-dev
RUN qmake 
RUN make 
RUN make install

WORKDIR /
RUN apt-get install -y ros-noetic-move-base-msgs
RUN apt-get install -y python3-catkin-tools
RUN apt-get install -y qtmultimedia5-dev

# ==================================================================
# Addition dependencies

RUN apt-get install -y ninja-build

CMD ["bash"]

