# docker/isaac_ros.Dockerfile
FROM nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_f247dd1051869171c3fc53bb35f6b907

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspaces/ros2_ws

RUN apt-get update && apt-get install -y \
    git \
    vim \
    tmux \
    wget \
    curl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
