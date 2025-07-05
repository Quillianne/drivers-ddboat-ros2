# DDBoat ROS 2 Drivers

This repository packages all the low‑level drivers that let a DDBoat talk to its
sensors and actuators through **ROS 2 Humble**.  
Everything is intended to run inside a single Docker image on a Raspberry Pi
(64‑bit OS recommended).

---

## Architecture

| Node (executable) | Purpose | Topic(s) |
|-------------------|---------|----------|
| `gps_node`        | Publishes GNSS position from the serial NMEA stream | `sensor_msgs/NavSatFix` |
| `arduino_node`    | Sends motor commands to the Arduino motor‑controller | subscribes `geometry_msgs/Twist` |
| `encoders_node`   | Publishes raw propeller‑encoder counts | `std_msgs/Int32MultiArray` |
| `imu_node`        | Publishes 9‑axis IMU data and calibrated heading; service `fast_heading_calibration` | `sensor_msgs/Imu`, `sensor_msgs/MagneticField`, `std_msgs/Float64` |
| `temperature_node`| Publishes motor temperatures from two TC74 sensors and exposes standby/config services | `sensor_msgs/Temperature`, diagnostics |
| `radio_node`      | Sends/receives LoRa packets (`id_src:id_dst:length:msg` frames) | `std_msgs/String` (`radio_tx`, `radio_rx`) |

The encoders node also provides two services:
`/clear_counts` resets the counters with the `C` command and `/request_last`
requests the previous reading using the `P` command. The polling delay can be
adjusted via the `delay` parameter which is sent to the device as `Dn;`.

A convenience launch file starts **all** of them at once:

```
ros2 launch ros2_ddboat all_nodes.launch.py
```

---

## Onboarding on Raspberry Pi

To set up Docker and our project on a fresh Raspberry Pi OS installation, run:

```bash
# Remove any conflicting container packages
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do
  sudo apt-get remove -y $pkg
done

# Add Docker's official GPG key
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/raspbian/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
  https://download.docker.com/linux/raspbian \
  $(. /etc/os-release && echo \"$VERSION_CODENAME\") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install Docker and related tools
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin

# Clone this repository and add your user to the docker group
git clone https://github.com/Quillianne/drivers-ddboat-ros2
cd drivers-ddboat-ros2
sudo usermod -aG docker $USER
newgrp docker  # apply group change without logout (optional)
```

you can then either build the DDBOAT Docker image or pull it.

### Configure image names

The build scripts and `docker-compose.yml` read the Docker repository names
from `.env` at the project root. Adjust these values if you push images to
your own registry:

```bash
IMAGE=quillianne/ddboat
ROS2_IMAGE=quillianne/ros2
```

---

## Build the Docker image

```bash
# from the repository root
docker build -t ddboat_ros2 .
```

For 64‑bit ARM boards build with the alternate file if you want a lighter image:

```bash
docker build -t ddboat_ros2 -f old_Dockerfile .
```

The image vendors the `wjwwood/serial` library, so no extra host packages are
needed.

---

## Running on a Raspberry Pi

# Using docker-compose

The repository ships with a `docker-compose.yml` that orchestrates the driver
containers and the optional WebSocket bridge.  Two profiles are provided:

* **`hw`** – run against the real Raspberry Pi hardware.
* **`sim`** – full software emulation for development on a laptop.

Run all drivers together with:

```bash
docker compose --profile hw up            # real hardware
# or
docker compose --profile sim up           # simulated devices
```

When using the *hardware* profile you can override which host devices are bound
into the containers by exporting environment variables before starting compose
(e.g. `GPS_DEV`, `ARDUINO_DEV`, … as documented in the compose file).

The `rosbridge` service (enabled in both profiles) exposes the ROS 2 graph on a
WebSocket port so that external applications can interact with the boat without
running ROS 2 natively.

---

## Talking to the boat with `roslibpy`

`roslibpy` is a small Python library that speaks the rosbridge protocol.  After
`docker compose` has started the `rosbridge` service you can publish and
subscribe to topics from anywhere on the network:

```python
import roslibpy

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

twist_pub = roslibpy.Topic(client, '/motors_cmd', 'geometry_msgs/Twist')
twist_pub.publish(roslibpy.Message({'linear': {'x': 50.0, 'y': 50.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}))
twist_pub.unadvertise()

client.terminate()
```

The scripts under `tests/` provide more complete examples that exercise all
drivers via rosbridge.

## Development tips

* Use `docker exec -it <container> bash` to enter a running boat and introspect
  topics with `ros2 topic echo …`.
