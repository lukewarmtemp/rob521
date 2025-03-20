#!/bin/bash

# Allow docker to connect to X server (run outside docker)
xhost +local:docker

docker run --platform linux/amd64 -it --user ros --network=host --ipc=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \
-e LIBGL_ALWAYS_SOFTWARE=1 \
-e QT_X11_NO_MITSHM=1 \
-v ./catkin_ws:/catkin_ws lukewarmtemp/rob521:lab01
