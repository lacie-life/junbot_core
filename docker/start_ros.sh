#!/bin/bash

xhost +

docker run -it --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$USER/:/home/$USER/:rw" \
    ros_noetic:05012024 \
    bash



