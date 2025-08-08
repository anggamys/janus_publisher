# Janus Publisher

## Overview

**Janus Publisher** is a ROS 2 package for streaming video from a camera device via the **Janus WebRTC Server**.
It uses **GStreamer** for media handling and offers flexible configuration through ROS 2 parameters.

## Installation

1. **Install GStreamer and its plugins**
   Run the following commands to install GStreamer and the required plugins:

   ```bash
   sudo apt-get update
   sudo apt-get install -y \
     python3-gi python3-gst-1.0 \
     gstreamer1.0-tools \
     gstreamer1.0-plugins-base \
     gstreamer1.0-plugins-good \
     gstreamer1.0-plugins-bad \
     gstreamer1.0-plugins-ugly
   ```

2. **Clone the package into your ROS 2 workspace**

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/anggamys/janus_publisher.git
   cd janus_publisher
   ```

3. **Build the package**

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select janus_publisher
   source install/setup.bash
   ```

## Usage

### Launch via ROS 2 launch file

To start the Janus Publisher using the default launch configuration:

```bash
ros2 launch janus_publisher streamer.launch.py
```

### Run the node directly

You can also run the node directly with custom parameters:

```bash
ros2 run janus_publisher streamer --ros-args \
  -p device:=/dev/video0 \
  -p host:=127.0.0.1 \
  -p rtp_port:=10000 \
  -p rtcp_port:=10001 \
  -p pt:=126 \
  -p width:=1280 \
  -p height:=720 \
  -p fps:=30 \
  -p bitrate:=2000 \
  -p keyint:=30
```
