#!/bin/bash

# Set image name (change if needed)
IMAGE="robobase:v1"
CONTAINER_NAME="robobase_dev"

# Use first argument as host repo path, or default to current directory
HOST_REPO_PATH="${1:-$(pwd)}"
CONTAINER_REPO_PATH="/root/robows/src/robobase"

# Shift to allow passing extra -v options as further arguments
shift || true

# X11 and user environment (host must provide XAUTH/XAUTHORITY)
XSOCK="/tmp/.X11-unix"
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"

# Check for NVIDIA GPU
if command -v nvidia-smi &> /dev/null; then
    GPU_OPTS="--runtime nvidia --gpus all \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all"
else
    GPU_OPTS=""
fi

# --privileged gives access to all host devices (USB, etc.)
docker run -v "${HOST_REPO_PATH}:${CONTAINER_REPO_PATH}" \
    -it \
    --privileged \
    --net=host \
    -v /dev/shm:/dev/shm \
    -v "${XSOCK}:${XSOCK}" \
    -v /dev/dri:/dev/dri \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="${XAUTH}" \
    -v "${XAUTH}:${XAUTH}" \
    --name "${CONTAINER_NAME}" \
    ${GPU_OPTS} \
    "$@" \
    "${IMAGE}"
