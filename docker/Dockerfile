FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

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
RUN cd qtmqtt
RUN git checkout v5.12.5
RUN apt-get -y install qtbase5-private-dev
RUN qmake 
RUN make 
RUN make install

WORKDIR /
RUN apt-get install -y ros-noetic-move-base-msgs
RUN apt-get install -y python3-catkin-tools
RUN apt-get install -y qtmultimedia5-dev








