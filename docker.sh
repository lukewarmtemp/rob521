#!/bin/bash

docker run --device /dev/snd \
-e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
--platform linux/amd64 -it --user ros --network=host --ipc=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \
-e LIBGL_ALWAYS_SOFTWARE=1 \
-v ./catkin_ws:/catkin_ws lukewarmtemp/rob521:lab01 \
