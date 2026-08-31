#!/usr/bin/python3
# -*- coding: utf-8 -*-
# NOTE: shebang is /usr/bin/python3 (not `env python3`) on purpose: this system
# has a conda Python 3.14 on PATH that cannot load the cp310 rclpy C extension.
"""Publish a birdview (aerial) image as a flat colored point cloud on the
ground plane so it can be used as a top-down map reference in RViz.

The PNG is decoded with the standard library only (zlib + struct), downsampled,
and projected onto the XY plane in the ``world`` frame. The resulting
``sensor_msgs/PointCloud2`` is latched (transient-local) on ``/birdview_cloud``
so RViz receives it whenever it subscribes (even after startup).

Parameters (set via config/birdview.yaml through the offboard launch file):
  image_path        absolute path to the PNG to display
  topic             output point cloud topic            (default: /birdview_cloud)
  frame_id          fixed frame of the overlay          (default: world)
  extent_x          world width  covered by the image [m], centered on origin
  extent_y          world height covered by the image [m], centered on origin
  z                 plane height of the overlay [m]     (default: 0.0)
  offset_x, offset_y  translation offset of the image [m]
  yaw               rotation of the image on the ground plane [rad]
  max_points        cap on the number of points after downsampling (raise for a
                    denser image when zooming; full 1920x1080 is ~2.07M)
  republish_period  re-publish period in seconds (0 = publish once)
"""

import math
import os
import struct
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def decode_png(path):
    """Decode an 8-bit non-interlaced PNG into (width, height, channels, raw).

    ``raw`` is a ``bytes`` object holding ``height * width * channels`` bytes in
    scanline order. Only the standard library is used.
    """
    with open(path, 'rb') as f:
        data = f.read()
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('not a PNG file: ' + path)

    pos = 8
    width = height = bit_depth = color_type = None
    palette = None
    idat = b''
    while pos < len(data):
        length = struct.unpack('>I', data[pos:pos + 4])[0]
        ctype = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if ctype == b'IHDR':
            (width, height, bit_depth, color_type,
             _comp, _filt, interlace) = struct.unpack('>IIBBBBB', chunk)
        elif ctype == b'PLTE':
            palette = chunk
        elif ctype == b'IDAT':
            idat += chunk
        elif ctype == b'IEND':
            break

    if bit_depth != 8:
        raise ValueError('only 8-bit PNG supported (got depth %d)' % bit_depth)
    if interlace != 0:
        raise ValueError('only non-interlaced PNG supported')

    # color types: 0 gray, 2 RGB, 3 palette, 4 gray+alpha, 6 RGBA
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(color_type)
    if channels is None:
        raise ValueError('unsupported PNG color type %d' % color_type)

    raw = zlib.decompress(idat)
    stride = width * channels
    out = bytearray(height * stride)
    prev = bytearray(stride)
    p = 0
    for y in range(height):
        ft = raw[p]
        p += 1
        line = bytearray(raw[p:p + stride])
        p += stride
        if ft == 1:  # Sub
            for i in range(channels, stride):
                line[i] = (line[i] + line[i - channels]) & 0xFF
        elif ft == 2:  # Up
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif ft == 3:  # Average
            for i in range(stride):
                a = line[i - channels] if i >= channels else 0
                line[i] = (line[i] + ((a + prev[i]) >> 1)) & 0xFF
        elif ft == 4:  # Paeth
            for i in range(stride):
                a = line[i - channels] if i >= channels else 0
                b = prev[i]
                c = prev[i - channels] if i >= channels else 0
                pp = a + b - c
                pa, pb, pc = abs(pp - a), abs(pp - b), abs(pp - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[i] = (line[i] + pr) & 0xFF
        out[y * stride:(y + 1) * stride] = line
        prev = line

    if color_type == 3:  # palette -> expand to RGB
        if not palette:
            raise ValueError('palette PNG missing PLTE chunk')
        rgb = bytearray(width * height * 3)
        for i, idx in enumerate(out):
            off = idx * 3
            rgb[i * 3:(i + 1) * 3] = palette[off:off + 3]
        channels = 3
        out = rgb

    return width, height, channels, bytes(out)


class BirdviewPublisher(Node):
    def __init__(self):
        super().__init__('birdview_publisher')
        self.declare_parameter('image_path', '')
        self.declare_parameter('topic', '/birdview_cloud')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('extent_x', 500.0)
        self.declare_parameter('extent_y', 300.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('max_points', 3000000)
        self.declare_parameter('republish_period', 10.0)

        self._msg = None
        image_path = self.get_parameter('image_path').value
        if not image_path or not os.path.isfile(image_path):
            self.get_logger().error(
                f'birdview image not found: {image_path!r} '
                '(set the image_path parameter)')
        else:
            try:
                width, height, channels, raw = decode_png(image_path)
                self._msg = self._build_cloud(width, height, channels, raw)
                self.get_logger().info(
                    f'birdview overlay: {width}x{height} image -> '
                    f'{self._msg.width} points on '
                    f'{self.get_parameter("frame_id").value} (z={self.get_parameter("z").value:.2f})')
            except Exception as exc:  # noqa: BLE001 - keep the node alive
                self.get_logger().error(f'failed to decode birdview image: {exc}')

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter('topic').value, qos)

        if self._msg is not None:
            period = float(self.get_parameter('republish_period').value)
            if period > 0:
                self.create_timer(period, self._publish)
            self._publish()

    def _build_cloud(self, width, height, channels, raw):
        extent_x = float(self.get_parameter('extent_x').value)
        extent_y = float(self.get_parameter('extent_y').value)
        z = float(self.get_parameter('z').value)
        offset_x = float(self.get_parameter('offset_x').value)
        offset_y = float(self.get_parameter('offset_y').value)
        yaw = float(self.get_parameter('yaw').value)
        max_points = max(int(self.get_parameter('max_points').value), 1)
        frame_id = self.get_parameter('frame_id').value

        x_min, x_max = -extent_x / 2.0, extent_x / 2.0
        y_min, y_max = -extent_y / 2.0, extent_y / 2.0
        denom_x = max(width - 1, 1)
        denom_y = max(height - 1, 1)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        # Downsample so the number of points stays under max_points.
        step = max(1, int(math.ceil(math.sqrt(float(width * height) / max_points))))

        stride = width * channels
        has_alpha = channels in (2, 4)
        buf = bytearray()
        n = 0
        for v in range(0, height, step):
            # Image top -> +y (north-up map convention).
            y0 = y_max - (v / denom_y) * extent_y
            row_off = v * stride
            for u in range(0, width, step):
                o = row_off + u * channels
                if has_alpha and raw[o + channels - 1] == 0:
                    continue  # skip fully transparent pixels
                r = raw[o]
                g = raw[o + 1] if channels >= 3 else r
                b = raw[o + 2] if channels >= 3 else r
                # Image left -> -x (west), then rotate (yaw) and translate.
                x0 = x_min + (u / denom_x) * extent_x
                xw = cos_yaw * x0 - sin_yaw * y0 + offset_x
                yw = sin_yaw * x0 + cos_yaw * y0 + offset_y
                buf += struct.pack('<fffI', xw, yw, z, (r << 16) | (g << 8) | b)
                n += 1

        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.data = bytes(buf)
        msg.is_dense = True
        return msg

    def _publish(self):
        if self._msg is None:
            return
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main():
    rclpy.init()
    node = BirdviewPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
