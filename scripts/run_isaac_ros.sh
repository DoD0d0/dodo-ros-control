# scripts/run_isaac_ros.sh
#!/usr/bin/env bash
set -e

CONTAINER_NAME=dodo_isaac_ros
IMAGE_NAME=dodo-isaac-ros:latest
WS_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/ros2_ws

# Allow X access (kept for future GUI use)
xhost +local:docker >/dev/null 2>&1 || true

# Attach if running
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  docker exec -it ${CONTAINER_NAME} bash
  exit 0
fi

# Start if exists but stopped
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  docker start ${CONTAINER_NAME}
  docker exec -it ${CONTAINER_NAME} bash
  exit 0
fi

# Create new container
docker run -it \
  --name ${CONTAINER_NAME} \
  --gpus all \
  --network=host \
  -v ${WS_DIR}:/workspaces/ros2_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  ${IMAGE_NAME} \
  bash
