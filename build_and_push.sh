#!/bin/bash

docker buildx build \
  --platform linux/arm/v7,linux/arm64 \
  -f Dockerfile \
  -t quillianne/ddboat:latest \
  -t quillianne/ddboat:armhf \
  -t quillianne/ddboat:arm64 \
  --push \
  .