#!/bin/bash

xhost +

docker run -it --privileged --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$USER/:/home/$USER/:rw" \
    aet_junbot:latest \
    bash



