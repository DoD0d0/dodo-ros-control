#!/bin/bash

# =========================
# Configuration
# =========================
IMAGE="dodo_ros_control:humble"

if [[ "$1" == "--gui" ]]; then
  IMAGE="dodo_ros_control:humble-gui"
fi

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
# Platform detection
# =========================
ARCH=$(uname -m)
if [[ "$ARCH" == "arm64" ]]; then
  PLATFORM="linux/arm64"
else
  PLATFORM="linux/amd64"
fi

# =========================
# X11 setup (Mac)
# =========================
export DISPLAY=:0

# Start XQuartz if not running
if ! pgrep -x "XQuartz" > /dev/null; then
  echo "[dodo] Starting XQuartz..."
  open -a XQuartz
  sleep 2
fi

# Allow X11 connections
xhost + >/dev/null 2>&1

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
    -e DISPLAY=host.docker.internal:0 \
    -e QT_X11_NO_MITSHM=1 \
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
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ${ROS_WS_HOST}:${ROS_WS_CONT}:rw \
  -v ${ASSETS_HOST}:${ASSETS_CONT}:rw \
  -v ${MODELS_HOST}:${MODELS_CONT}:rw \
  ${IMAGE} \
  bash