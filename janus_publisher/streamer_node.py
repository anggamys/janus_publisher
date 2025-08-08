#!/usr/bin/env python3
import signal
import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node

class StreamerNode(Node):
    def __init__(self):
        super().__init__('janus_cam_streamer')

        # Declare ROS parameters (dengan default aman)
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('rtp_port', 10000)
        self.declare_parameter('rtcp_port', 10001)
        self.declare_parameter('pt', 126)              # H264 PT
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('bitrate', 2000)        # kbps
        self.declare_parameter('keyint', 30)
        self.declare_parameter('use_vaapi', False)     # true kalau mau HW encode Intel

        # Ambil param
        p = self.get_parameter
        self.dev     = p('device').get_parameter_value().string_value
        self.host    = p('host').get_parameter_value().string_value
        self.rtp     = p('rtp_port').get_parameter_value().integer_value
        self.rtcp    = p('rtcp_port').get_parameter_value().integer_value
        self.pt      = p('pt').get_parameter_value().integer_value
        self.width   = p('width').get_parameter_value().integer_value
        self.height  = p('height').get_parameter_value().integer_value
        self.fps     = p('fps').get_parameter_value().integer_value
        self.bitrate = p('bitrate').get_parameter_value().integer_value
        self.keyint  = p('keyint').get_parameter_value().integer_value
        self.vaapi   = p('use_vaapi').get_parameter_value().bool_value

        Gst.init(None)
        self.pipeline = self._build_pipeline()
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self._on_msg)

        self.get_logger().info(
            f"Streaming {self.dev} → {self.host}:{self.rtp}/{self.rtcp} "
            f"(PT={self.pt}, {self.width}x{self.height}@{self.fps}fps, {self.bitrate}kbps, "
            f"{'VAAPI' if self.vaapi else 'x264'})"
        )

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Failed to set pipeline to PLAYING")
            rclpy.shutdown()

    def _build_pipeline(self):
        caps = f"video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1"
        if self.vaapi:
            # HW encode (Intel); pastikan gstreamer1.0-vaapi sudah terpasang
            pipe = f"""
                rtpbin name=rtpbin
                v4l2src device={self.dev} do-timestamp=true !
                  videoconvert ! videoscale ! {caps} !
                  vaapih264enc rate-control=cbr bitrate={self.bitrate} key-int={self.keyint} tune=low-power !
                  h264parse config-interval=-1 !
                  rtph264pay pt={self.pt} !
                  rtpbin.send_rtp_sink_0
                rtpbin.send_rtp_src_0  ! udpsink host={self.host} port={self.rtp}
                rtpbin.send_rtcp_src_0 ! udpsink host={self.host} port={self.rtcp} sync=false async=false
            """
        else:
            # Software x264 → Baseline (kompatibel dengan janus videofmtp=42e01f)
            pipe = f"""
                rtpbin name=rtpbin
                v4l2src device={self.dev} do-timestamp=true !
                  videoconvert ! videoscale ! {caps} !
                  x264enc tune=zerolatency speed-preset=veryfast bitrate={self.bitrate} key-int-max={self.keyint} byte-stream=false !
                  video/x-h264,profile=baseline !
                  h264parse config-interval=-1 !
                  rtph264pay pt={self.pt} !
                  rtpbin.send_rtp_sink_0
                rtpbin.send_rtp_src_0  ! udpsink host={self.host} port={self.rtp}
                rtpbin.send_rtcp_src_0 ! udpsink host={self.host} port={self.rtcp} sync=false async=false
            """
        return Gst.parse_launch(pipe)

    def _on_msg(self, bus, msg):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            self.get_logger().error(f"GStreamer ERROR: {err}")
            if dbg:
                self.get_logger().error(dbg)
            self.pipeline.set_state(Gst.State.NULL)
            rclpy.shutdown()
        elif t == Gst.MessageType.EOS:
            self.get_logger().info("End of Stream")
            self.pipeline.set_state(Gst.State.NULL)
            rclpy.shutdown()

    def destroy_node(self):
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StreamerNode()

    # Tangkap Ctrl+C agar rapi
    def _sigint_handler(sig, frame):
        node.get_logger().info("SIGINT received, shutting down...")
        node.destroy_node()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
