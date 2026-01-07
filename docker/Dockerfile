FROM ros:humble

# Use bash
SHELL ["/bin/bash", "-c"]

# Basic tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set ROS workspace
ENV ROS_WS=/ros2_ws
WORKDIR ${ROS_WS}

# Copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]