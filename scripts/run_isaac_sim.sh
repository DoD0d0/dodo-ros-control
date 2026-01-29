#!/usr/bin/env bash
set -e

IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0
CONTAINER=dodo_isaac_sim

xhost +local:docker >/dev/null 2>&1 || true

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  docker rm -f ${CONTAINER}
fi

docker run -it \
  --name ${CONTAINER} \
  --gpus all \
  --network=host \
  --ipc=host \
  -e ACCEPT_EULA=Y \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ${IMAGE} \
  ./runapp.sh
