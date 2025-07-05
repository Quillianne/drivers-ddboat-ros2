#!/bin/bash

docker buildx build \
  --platform linux/arm/v7,linux/arm64 \
  -f Dockerfile \
  -t quillianne/ros2:latest \
  -t quillianne/ros2:armhf \
  -t quillianne/ros2:arm64 \
  --push \
  .