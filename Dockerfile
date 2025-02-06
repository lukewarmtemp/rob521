FROM osrf/ros:noetic-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
  mesa-utils \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  python3-colcon-argcomplete

# Install text editors
RUN apt-get update && apt-get install -y \
  nano \
  vim

# Install ROS packages for TurtleBot
RUN sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
ros-noetic-rosserial-python ros-noetic-rosserial-client \
ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
ros-noetic-compressed-image-transport ros-noetic-rqt* \
ros-noetic-rviz ros-noetic-gmapping \
ros-noetic-navigation ros-noetic-interactive-markers -y

# Install TurtleBot3 packages
RUN sudo apt install ros-noetic-dynamixel-sdk -y
RUN sudo apt install ros-noetic-turtlebot3-msgs -y
RUN sudo apt install ros-noetic-turtlebot3 -y

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-yaml \
    python3-pygame \
    python3-matplotlib \
    python3-skimage \
    python3-scipy

# Ensure that pygame uses the correct display (for headless use)
RUN apt-get update && apt-get install -y \
    libx11-dev \
    libxrender-dev \
    libxext-dev

RUN pip install --upgrade scikit-image
RUN apt-get update && apt-get install -y \
    alsa-utils \
    pulseaudio \
    python3-tkinter

# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
RUN echo 'source /opt/ros/noetic/setup.bash' >> /home/${USERNAME}/.bashrc
RUN echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> /home/${USERNAME}/.bashrc
RUN echo 'source /opt/ros/noetic/setup.bash' >> /home/${USERNAME}/.bashrc
RUN echo 'source /catkin_ws/devel/setup.bash' >> /home/${USERNAME}/.bashrc

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]

USER root
RUN echo 'export TURTLEBOT3_MODEL=waffle_pi' >> /home/${USERNAME}/.bashrc
USER $USERNAME