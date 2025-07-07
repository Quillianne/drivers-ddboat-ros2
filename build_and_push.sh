#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/.env"


IMAGE_NAME="${IMAGE}"
ROS2_IMAGE="${ROS2_IMAGE}"
TAG="latest"

# pulling the latest ROS2 image
echo "🔄 Pulling latest base ROS2 image: $ROS2_IMAGE:$TAG..."
docker pull $ROS2_IMAGE:$TAG

echo "🚧 Building multiarch image for $IMAGE_NAME:$TAG..."

docker buildx build \
  --platform linux/arm/v7,linux/arm64 \
  -t $IMAGE_NAME:$TAG \
  --push \
  .

echo "🔍 Fetching architecture-specific digests..."

MANIFEST_JSON=$(docker buildx imagetools inspect $IMAGE_NAME:$TAG --raw)

DIGEST_ARMHF=$(echo "$MANIFEST_JSON" | jq -r '.manifests[] | select(.platform.architecture=="arm" and .platform.variant=="v7") | .digest')
DIGEST_ARM64=$(echo "$MANIFEST_JSON" | jq -r '.manifests[] | select(.platform.architecture=="arm64") | .digest')

echo "✅ armhf digest: $DIGEST_ARMHF"
echo "✅ arm64 digest: $DIGEST_ARM64"

if [[ -z "$DIGEST_ARMHF" || -z "$DIGEST_ARM64" ]]; then
  echo "❌ Failed to extract digests. Check the image name and tags."
  exit 1
fi


echo "📦 Creating tag for armhf..."
docker manifest create $IMAGE_NAME:armhf --amend $IMAGE_NAME@$DIGEST_ARMHF
docker manifest push $IMAGE_NAME:armhf

echo "📦 Creating tag for arm64..."
docker manifest create $IMAGE_NAME:arm64 --amend $IMAGE_NAME@$DIGEST_ARM64
docker manifest push $IMAGE_NAME:arm64

echo "🎉 Done! Multiarch image and architecture-specific tags pushed successfully."