FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
        gazebo \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        ros-humble-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws/src

COPY fastbot_msgs ./fastbot_msgs
COPY fastbot_description ./fastbot_description
COPY fastbot_gazebo ./fastbot_gazebo
COPY fastbot_waypoints ./fastbot_waypoints

WORKDIR /root/ros2_ws
RUN bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --event-handlers console_direct+"

CMD ["bash"]