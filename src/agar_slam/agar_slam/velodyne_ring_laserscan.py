#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


class VelodyneRingLaserScan(Node):

    def __init__(self):
        super().__init__('velodyne_ring_laserscan')

        self.declare_parameter('rings', [7, 8, 9])
        self.declare_parameter('resolution', 0.007)
        self.declare_parameter('range_min', 0.0)
        self.declare_parameter('range_max', 200.0)

        self.rings = np.asarray(
            sorted(set(int(value) for value in self.get_parameter('rings').value)),
            dtype=np.uint16,
        )
        self.resolution = abs(float(self.get_parameter('resolution').value))
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        if self.rings.size == 0:
            raise ValueError('At least one Velodyne ring must be selected')
        if self.resolution <= 0.0:
            raise ValueError('resolution must be greater than zero')
        if self.range_min < 0.0 or self.range_max <= self.range_min:
            raise ValueError('Invalid range_min/range_max values')

        self.bin_count = round(2.0 * math.pi / self.resolution)
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(LaserScan, 'scan', scan_qos)
        self.subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.pointcloud_callback,
            scan_qos,
        )
        self.message_count = 0
        self.get_logger().info(
            'Velodyne ring scan ready: rings=%s bins=%d'
            % (self.rings.tolist(), self.bin_count)
        )

    @staticmethod
    def field_map(msg):
        return {field.name: field for field in msg.fields}

    @staticmethod
    def strided_field(msg, field, count):
        if field.datatype == PointField.FLOAT32:
            dtype = '>f4' if msg.is_bigendian else '<f4'
        elif field.datatype == PointField.UINT16:
            dtype = '>u2' if msg.is_bigendian else '<u2'
        else:
            raise ValueError(
                "Unsupported datatype %d for PointCloud2 field '%s'"
                % (field.datatype, field.name)
            )

        return np.ndarray(
            shape=(count,),
            dtype=np.dtype(dtype),
            buffer=msg.data,
            offset=field.offset,
            strides=(msg.point_step,),
        )

    def pointcloud_callback(self, msg):
        fields = self.field_map(msg)
        missing = [name for name in ('x', 'y', 'ring') if name not in fields]
        if missing:
            self.get_logger().error(
                'PointCloud2 is missing fields: %s' % ', '.join(missing)
            )
            return

        expected_size = msg.row_step * msg.height
        if msg.point_step <= 0 or len(msg.data) < expected_size:
            self.get_logger().error('Invalid PointCloud2 data layout')
            return
        if msg.height > 1 and msg.row_step != msg.point_step * msg.width:
            self.get_logger().error(
                'PointCloud2 row padding is not supported by this offline tool'
            )
            return

        count = msg.width * msg.height
        try:
            x = self.strided_field(msg, fields['x'], count)
            y = self.strided_field(msg, fields['y'], count)
            ring = self.strided_field(msg, fields['ring'], count)
        except (TypeError, ValueError) as ex:
            self.get_logger().error(str(ex))
            return

        selected = np.isin(ring, self.rings)
        selected &= np.isfinite(x) & np.isfinite(y)
        if not np.any(selected):
            self.publish_scan(msg, np.empty(0, dtype=np.int32), np.empty(0))
            return

        selected_x = x[selected]
        selected_y = y[selected]
        ranges = np.hypot(selected_x, selected_y)
        valid = (ranges >= self.range_min) & (ranges <= self.range_max)
        ranges = ranges[valid]
        angles = np.arctan2(selected_y[valid], selected_x[valid])
        bins = ((angles + math.pi) / self.resolution).astype(np.int32)
        inside = (bins >= 0) & (bins < self.bin_count)
        self.publish_scan(msg, bins[inside], ranges[inside])

    def publish_scan(self, cloud, bins, point_ranges):
        ranges = np.full(self.bin_count, np.inf, dtype=np.float32)
        if bins.size:
            np.minimum.at(ranges, bins, point_ranges)

        scan = LaserScan()
        scan.header = cloud.header
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.resolution
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.publisher.publish(scan)

        self.message_count += 1
        if self.message_count == 1:
            self.get_logger().info('Published first selected-ring LaserScan.')


def main(args=None):
    rclpy.init(args=args)
    node = VelodyneRingLaserScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(
            'Velodyne ring scan finished: processed={}'.format(node.message_count),
            flush=True,
        )
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
