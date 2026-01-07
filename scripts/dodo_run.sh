#!/bin/bash

# =========================
# Platform detection
# =========================
ARCH=$(uname -m)
if [[ "$ARCH" == "arm64" ]]; then
  PLATFORM="linux/arm64"
else
  PLATFORM="linux/amd64"
fi

# =========================
# Configuration
# =========================
IMAGE="dodo_ros_control:humble"
CONTAINER_NAME="dodo_ros_control"

# Repo root (absolute path)
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_WS_HOST="${REPO_ROOT}/ros2_ws"
ASSETS_HOST="${REPO_ROOT}/assets"
MODELS_HOST="${REPO_ROOT}/models"

ROS_WS_CONT="/ros2_ws"
ASSETS_CONT="/assets"
MODELS_CONT="/models"

# =========================
# Docker logic
# =========================

# If container exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[dodo] Container already running. Attaching..."
  else
    echo "[dodo] Container exists but stopped. Starting..."
    docker start ${CONTAINER_NAME} >/dev/null
  fi

  docker exec -it \
    -w ${ROS_WS_CONT} \
    ${CONTAINER_NAME} \
    bash
  exit 0
fi

# =========================
# Run new container
# =========================
echo "[dodo] Starting new container..."

docker run -it \
  --platform ${PLATFORM} \
  --name ${CONTAINER_NAME} \
  -v ${ROS_WS_HOST}:${ROS_WS_CONT}:rw \
  -v ${ASSETS_HOST}:${ASSETS_CONT}:rw \
  -v ${MODELS_HOST}:${MODELS_CONT}:rw \
  dodo_ros_control:humble \
  bash