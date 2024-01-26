FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY ./ros_ws/src /opt/jugglebot/src

WORKDIR /opt/jugglebot
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths ./src -y --ignore-src

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

RUN rm -rf /opt/jugglebot/src

COPY ./docker/ros/entrypoint.sh /

ENTRYPOINT [ "/entrypoint.sh" ]
