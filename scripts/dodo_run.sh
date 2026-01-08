#!/bin/bash

IMAGE="dodo_ros_control:humble"
if [[ "$1" == "--gui" ]]; then
  IMAGE="dodo_ros_control:humble-gui"
fi

CONTAINER_NAME="dodo_ros_control"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_WS_HOST="${REPO_ROOT}/ros2_ws"
ASSETS_HOST="${REPO_ROOT}/assets"
MODELS_HOST="${REPO_ROOT}/models"

ROS_WS_CONT="/ros2_ws"
ASSETS_CONT="/assets"
MODELS_CONT="/models"

ARCH=$(uname -m)
if [[ "$ARCH" == "arm64" ]]; then
  PLATFORM="linux/arm64"
else
  PLATFORM="linux/amd64"
fi

# =========================
# Docker logic
# =========================

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[dodo] Container already running. Attaching..."
  else
    echo "[dodo] Container exists but stopped. Starting..."
    docker start ${CONTAINER_NAME} >/dev/null
  fi

  docker exec -it ${CONTAINER_NAME} \
    bash -lc "source /opt/ros/humble/setup.bash && \
              [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash; \
              exec bash"
  exit 0
fi

echo "[dodo] Starting new container..."

docker run -it \
  --platform ${PLATFORM} \
  --name ${CONTAINER_NAME} \
  -v ${ROS_WS_HOST}:${ROS_WS_CONT}:rw \
  -v ${ASSETS_HOST}:${ASSETS_CONT}:rw \
  -v ${MODELS_HOST}:${MODELS_CONT}:rw \
  ${IMAGE}