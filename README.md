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
| `imu_node`        | Publishes 9‑axis IMU data (mag/gyro/accel) | `sensor_msgs/Imu`, `sensor_msgs/MagneticField` |
| `temperature_node`| Publishes motor temperatures from two TC74 sensors | `sensor_msgs/Temperature` |
| `radio_node`      | Sends/receives LoRa packets | `std_msgs/String` (`radio_tx`, `radio_rx`) |

A convenience launch file starts **all** of them at once:

```
ros2 launch ros2_ddboat all_nodes.launch.py
```

---

## Build the Docker image

```bash
# from the repository root
docker build -t ddboat_ros2 .
```

The image vendors the `wjwwood/serial` library, so no extra host packages are
needed.

---

## Running on a Raspberry Pi

### With real hardware attached

```bash
docker run --rm -it --privileged \
  --device=/dev/ttyGPS0 \   # GPS
  --device=/dev/ttyACM0 \   # Arduino motor controller
  --device=/dev/ttyUSB0 \   # Encoders (example)
  --device=/dev/i2c-1 \     # IMU + temperature sensors
  ddboat_ros2 \
  ros2 launch ros2_ddboat all_nodes.launch.py
```

`--privileged` passes the USB, serial, and I²C devices straight through so the
drivers can open them.

### Without any hardware (quick simulation)

```bash
docker run --rm -it --privileged ddboat_ros2 bash -c '
  source /opt/ros/humble/setup.bash
  source /opt/ws/install/local_setup.bash
  ros2 run ros2_ddboat emulate_devices.py &          # create fake /dev/tty* ports and /dev/i2c-1
  ros2 launch ros2_ddboat all_nodes.launch.py        # start every driver
'
```

The `emulate_devices.py` helper keeps a set of pseudo‑terminals open and feeds
dummy data. It now also emulates `/dev/i2c-1` so the IMU and temperature nodes
receive random values even without real sensors.

---

## Running the Python tests

Each test script in `tests/` talks to one driver via ROS 2 messages.  
Mount the folder read‑only and run the desired test inside the same image:

```bash
docker run --rm -it -v "$PWD/tests":/tests:ro ddboat_ros2 \
  bash -c "source /opt/ros/humble/setup.bash && \
           python3 /tests/test_gps_node.py"
```

Replace `test_gps_node.py` with any other script to exercise a different node.

---

## Development tips

* Use `docker exec -it <container> bash` to enter a running boat and introspect
  topics with `ros2 topic echo …`.

