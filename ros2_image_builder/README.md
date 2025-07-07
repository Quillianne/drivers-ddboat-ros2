# DDBoat ROS 2 base image Build Guide

This guide walks you through building a Docker image for ROS 2 Humble on 32-bit ARM (armhf) and multi-architecture variants using Docker Buildx, then running it on a host or pushing it to a registry.
On my M4 PRO, building took approximatively 40 minutes for armhf (and 4min for arm64)

It is better to build them on a PC and not on the raspberry pi

The `build_and_push.sh` script reads the target repository from `../.env`.
Edit `ROS2_IMAGE` in that file if you plan to push to your own registry.

---

## Prerequisites

1. **Docker & Buildx**  
   - Docker ≥ 20.10  
   - Buildx enabled (bundled with Docker Desktop; on Linux see Docker docs).

2. **QEMU emulators (optional, for emulation / multi arch)**  
   ```bash
   docker run --privileged --rm tonistiigi/binfmt --install all
   ```
   This registers QEMU handlers for non-native architectures.

3. **(Optional) Git**  
   Needed if you clone sources or repos:
   ```bash
   sudo apt update && sudo apt install -y git
   ```

---

## 1. Create or switch to a Buildx builder

> Use an isolated container-based builder to support ARM builds:

```bash
docker buildx create --name rpi_builder \
  --driver docker-container \
  --bootstrap \
  --use
```

Verify supported platforms:

```bash
docker buildx inspect --bootstrap | grep Platforms
# Expected to list: linux/amd64, linux/arm/v7, linux/arm64, ...
```

---

## 2. Build multi-arch images

You have to create a ros2 repo in DockerHub repo or any other name but you have to modify following commands.
Build both armhf (arm/v7) and arm64 variants and push to a registry:

!!! Be aware to be in your ros2_mini_docker directory !!!

```bash
docker buildx build \
  --platform linux/arm/v7,linux/arm64 \
  -f Dockerfile \
  -t yourusername/ros2:latest \
  -t yourusername/ros2:armhf \
  -t yourusername/ros2:arm64 \
  --push \
  .
```

- `--platform`: target platforms
- `-f`: path to Dockerfile
- `-t`: tag for each variant
- `--push`: pushes the resulting manifest list and images

> **Note:** If you want to load one variant locally instead of pushing:
> ```bash
> docker buildx build \
>   --platform linux/arm/v7 \
>   -f Dockerfile \
>   -t quillianne/ros2:armhf \
>   --load \
>   .
> ```
> Repeat with `--load` per platform if needed.

---

## 3. Using the image

### Pulling

On an ARMv7 host (e.g., Raspberry Pi):

```bash
docker pull quillianne/ros2:armhf
```

On an ARM64 host:

```bash
docker pull quillianne/ros2:arm64
```

On any host (auto-select variant based on host architecture):

```bash
docker pull quillianne/ros2
```

### Running

```bash
docker run --rm -it \
  --platform linux/arm/v7 \
  quillianne/ros2:armhf \
  bash
```

Replace `--platform` and tag for arm64.

---

## 4. Troubleshooting

- **`pull access denied`**: ensure you’ve pushed `quillianne/ros2:armhf` to your registry.  
- **Image not found for arch**: inspect manifest list with:
  ```bash
  docker manifest inspect quillianne/ros2
  ```
- **BuildKit errors**: fallback to classic builder:
  ```bash
  DOCKER_BUILDKIT=0 docker build -f Dockerfile -t myimage:tag .
  ```

---

## 5. Additional useful commands

- **List Buildx builders**:
  ```bash
  docker buildx ls
  ```
- **Remove a builder**:
  ```bash
  docker buildx rm rpi_builder
  ```
- **Clean dangling images**:
  ```bash
  docker image prune -f
  ```
- **View logs during build**:
  ```bash
  docker buildx build --progress=plain ...
  ```