# DDBOAT python3 drivers version 2

The drivers are :
* IMU (MAG, ACCEL, GYRO) : imu9_driver_v2.py
* GPS (serial line, GPGLL message) : gps_driver_v2.py
* Encoders on propeller rotation (serial line) : encoders_driver_v2.py
* Arduino motors command (serial line) : arduino_driver_v2.py
* TC74 temperature sensors (one per motor) : tc74_driver_v2.py
* Simple send/receive protocol with LORA radio : radio_driver_V2.py


## testing the drivers on your ddboat

First, you will have to clone this repo on your laptop in a folder called **drivers** 
```
$ git clone https://gitlab.ensta-bretagne.fr/zerrbe/drivers-ddboat-v2.git drivers
```

This section should be rewritten to adapt with DDBoats V2 (todo)

### simplify access to ddboats

To avoid typing password every time, a ssh keys pair can be created :
```
cd ~/.ssh
ssh-keygen -t rsa
```
Filename can be ddboat_key and no passphrase is provided.
After creation the public key has to be copied on the DDBOAT with XX the boat number on 2 digit (e.g 07 for boat 7)
```
scp ddboat_key.pub ue32@172.20.25.2XX:.ssh
```
add the content of ddboat_key.pub at the end of **authorized_keys** (in ~/.ssh) using vi or nano.

Note : if **authorized_keys** does not exist in .ssh, just create it as an empty file :
```
touch authorized_keys
```


## using the screen command to work outside of the WIFI Network

When using WIFI, the terminal is locked when the WIFI connection is lost.
The ddboat wil stop working properly.
A solution is to use the **screen** command to detach the session from the terminal.
First, we start a screen session called sesddboat (or whatsoever name)

```
$ screen -S sesddboat
```
Note that screen creates a new terminal (and you loose the command history for recent commands)
We can now check that the session is attached
```
$ screen -ls
```
should give :
```
There is a screen on:
        1404.sesddboat     (09/29/21 16:42:04)     (Attached)
1 Socket in /run/screen/S-pi.

```
The screen commands start with Ctrl+A. You can see a list of usefull commands here :

https://linuxize.com/post/how-to-use-linux-screen/

Now, we can detach the screen session from the terminal by typing Ctrl+A d
and we can check that the session is actually detached :
```
$ screen -ls
```
should give :
```
There is a screen on:
	1404.sesddboat	(09/29/21 16:42:04)	(Detached)
1 Socket in /run/screen/S-pi.
```

Now the ddboat can work outside the WIFI coverage. To get access back (resume access) to the session when the ddboat is back in WIFI area, type :
```
screen -r
```
we are back on line with the ddboat !

A last usefull command is to fully stop the screen command, one way to do it is :

```
$ screen -X -S sesddboat quit
```

where sesddboat is the session name the has been used at the start.

Example (log the GPS in a gpx file)
```
$ screen -S sesgps
$ python3 tst_gpx.py (in the drivers-ddboat-v2 folder)
$ CTRL+A d  
now you can go outside the WIFI area and acquire a GPS track
when it's done, recover the session
$ screen -r
$ CTRL+C to stop the acquisition
a file called tst.gpx has been created, rename it to save the data
$ mv tst.gpx mynicetrack.gpx
we can now destroy the session
$ screen -X -S  sesgps quit
```


## working with GPS

The following packets must have been installed :
```
$ sudo apt install python3-gpxpy python3-pyproj
```

The GPS is acquired and the GPGLL message is recorded in GPX format (so you can plot it in GeoPortail for example) in the **tst.gpx** file with the command:
```
$ python3 tst_gpx.py
```

In the NMEA message GPGLL, the latitude and longitude are given in DDmm.mmmm with DD in degrees and mm.mmmm in decimal minutes. To get the value in decimal degrees we do DD + mm.mmmm/60.0

Using a projection from degrees (longitude,latitude in WGS84) to meters (UTM zone 30N) we can compute the distance in meter to a reference point. The following command executes a test:
```
$ python3 tst_proj.py
```

The gpx file can be display on **GeoPortail**. To show it on **GoogleEarth** the simpliest way is to convert it in kml :
```
$ gpsbabel -i gpx -f file.gpx -o kml -F file.kml


## remote install drivers for tests

From a remote computer use rcp and rsh , example for DDBOAT 5
```
$ scp install.bash pi@172.20.25.205:/tmp/
$ rsh 172.20.25.205 -l pi /tmp/install.bash
```





## ROS2 drivers

The `ros2_ddboat` package exposes the DDBoat hardware through ROS2 topics.  All
drivers are implemented as C++ nodes so they run efficiently on the
Raspberry Pi and inside the container:

- `gps_node` – publishes `sensor_msgs/NavSatFix` from the GPS serial port.
- `arduino_node` – subscribes to `motors_cmd` (`geometry_msgs/Twist`) and sends
  commands to the motor controller using the Arduino serial link.
- `encoders_node` – publishes raw encoder values on `encoders`
  (`std_msgs/Int32MultiArray`).
- `imu_node` – reads the 9‑axis IMU over I²C and publishes data on `imu/data_raw`
  and `imu/mag`.
- `temperature_node` – reads the two TC74 temperature sensors and publishes them
  on `temperature_left` and `temperature_right`.
- `radio_node` – sends and receives LoRa messages on `radio_tx` and `radio_rx`.

The legacy Python drivers use dedicated device names which are also the
defaults for the ROS2 nodes:

- `/dev/ttyGPS[0-2]` – virtual GPS serial ports
- `/dev/ttyV0` or `/dev/ttyACM0` – Arduino motor controller
- `/dev/ttyENC[0-2]` (or `/dev/ttyENCRAW`) – encoder interface
- `/dev/ttyLORA[1-2]` – LoRa radio
- `/dev/i2c-1` – I²C bus for the IMU and temperature sensors

When hardware is unavailable you can emulate the serial devices with
`ros2 run ros2_ddboat emulate_devices.py`.  Running it as root will
create symbolic links in `/dev/` so the nodes can open the usual
paths:

```bash
sudo ros2 run ros2_ddboat emulate_devices.py
```

### Build

The repository vendors the [wjwwood/serial](https://github.com/wjwwood/serial)
library so no extra dependencies are needed.  Simply build the package with
`colcon`:

```bash
colcon build --packages-select ros2_ddboat
```

### Run

Launch any node and set parameters if required. Example commands:

```bash
ros2 run ros2_ddboat gps_node --ros-args -p port:=/dev/ttyGPS0 -p baud:=9600
ros2 run ros2_ddboat arduino_node
ros2 run ros2_ddboat encoders_node
ros2 run ros2_ddboat imu_node
ros2 run ros2_ddboat temp_node
ros2 run ros2_ddboat radio_node
```

When running inside a container, share the serial devices (e.g. `/dev/ttyGPS0`, `/dev/ttyV0`) with the container using the `--device` option or with `--privileged` mode to allow access from the node.

### Docker workflow

Build the image and start a container that has access to the hardware devices:

```bash
docker build -t ddboat_ros2 .
docker run --rm -it \
  --device=/dev/ttyGPS0 \
  --device=/dev/ttyV0 \
  ddboat_ros2
```

### Docker Compose

To run all nodes at once use the provided compose file:

```bash
docker compose up --build
```

This starts every driver using the launch file
`ros2_ddboat/launch/all_nodes.launch.py`.

The entrypoint automatically sources the ROS environment and the workspace,
via `/entrypoint.sh`, so you can run the nodes right away:

```bash
ros2 run ros2_ddboat arduino_node
```

#### Raspberry Pi setup

On a real Raspberry Pi you can run the same image.  Map the hardware
serial and I²C devices with `--device` so the nodes can access them.
When developing on a PC or in a virtual machine you may create fake
devices for testing:

```bash
sudo ros2 run ros2_ddboat emulate_devices.py
```

This command creates `/dev/ttyGPS0`, `/dev/ttyV0`, and other dummy
serial ports that mirror each other so the ROS2 nodes run without the
actual sensors.

### Python tests

The `tests` directory contains small Python scripts that can be run on the host to
check each ROS2 driver. Start the desired nodes first (inside the container or on
the Pi) and then run for example:

```bash
python3 tests/test_gps_node.py
python3 tests/test_arduino_node.py
python3 tests/test_encoders_node.py
python3 tests/test_imu_node.py
python3 tests/test_temp_node.py
python3 tests/test_radio_node.py
```

Each script uses `rclpy` to send or receive messages to the running node and will
exit after a message is exchanged.
