"""Publish a ray-cast VLP-16 cloud and isolated simulation TF frames."""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from cat_terrain.scene import TerrainScene


def quaternion(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = [math.radians(value) / 2 for value in
                        (roll_deg, pitch_deg, yaw_deg)]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class TerrainPointcloudGenerator(Node):
    def __init__(self):
        super().__init__('terrain_pointcloud_generator', namespace='sim')
        scene_file = self.declare_parameter('scene_file', '').value
        if not scene_file:
            raise ValueError('scene_file parameter is required')
        self.scene = TerrainScene.from_file(scene_file)
        self.start_time = time.monotonic()
        self.rng = np.random.default_rng(487)
        self.publisher = self.create_publisher(PointCloud2, 'velodyne_points', 1)
        marker_qos = QoSProfile(depth=1,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'scene_markers', marker_qos)
        self.static_tf = StaticTransformBroadcaster(self)
        self.dynamic_tf = TransformBroadcaster(self)
        self.publish_static_tf()
        self.publish_scene_markers()
        self.marker_timer = self.create_timer(1.0, self.publish_scene_markers)
        self.timer = self.create_timer(1.0 / self.scene.rate_hz, self.publish_scan)
        self.get_logger().info(
            f'Synthetic VLP-16: {scene_file}; {self.scene.azimuth_samples} azimuths, '
            f'{self.scene.rate_hz:g} Hz; /sim/velodyne_points')

    def publish_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'sim_base_link'
        transform.child_frame_id = 'sim_vlp16'
        transform.transform.translation.x = float(self.scene.sensor_xyz[0])
        transform.transform.translation.y = float(self.scene.sensor_xyz[1])
        transform.transform.translation.z = float(self.scene.sensor_xyz[2])
        rotation = quaternion(*self.scene.sensor_rpy_deg)
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = rotation
        self.static_tf.sendTransform(transform)

    def publish_scene_markers(self):
        markers = MarkerArray()
        for index, (bounds, height) in enumerate(
                self.scene.visible_surface_patches()):
            marker = Marker()
            marker.header.frame_id = 'sim_odom'
            marker.ns = 'floor'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float((bounds[0] + bounds[1]) / 2)
            marker.pose.position.y = float((bounds[2] + bounds[3]) / 2)
            marker.pose.position.z = height - 0.005
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(bounds[1] - bounds[0])
            marker.scale.y = float(bounds[3] - bounds[2])
            marker.scale.z = 0.01
            marker.color.r = 0.2
            marker.color.g = 0.5 if height >= 0 else 0.3
            marker.color.b = 0.6 if height >= 0 else 0.9
            marker.color.a = 0.25
            markers.markers.append(marker)
        for index, (center, size, yaw) in enumerate(self.scene.boxes):
            marker = Marker()
            marker.header.frame_id = 'sim_odom'
            marker.ns = 'boxes'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            rotation = quaternion(0, 0, yaw)
            (marker.pose.orientation.x, marker.pose.orientation.y,
             marker.pose.orientation.z, marker.pose.orientation.w) = rotation
            marker.scale.x = float(size[0])
            marker.scale.y = float(size[1])
            marker.scale.z = float(size[2])
            marker.color.r = 0.95
            marker.color.g = 0.5
            marker.color.b = 0.1
            marker.color.a = 0.35
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)

    def publish_scan(self):
        pose = self.scene.pose_at(time.monotonic() - self.start_time)

        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'sim_odom'
        transform.child_frame_id = 'sim_base_link'
        transform.transform.translation.x = float(pose[0])
        transform.transform.translation.y = float(pose[1])
        transform.transform.translation.z = float(pose[2])
        rotation = quaternion(0, 0, pose[3])
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = rotation
        self.dynamic_tf.sendTransform(transform)

        scan = self.scene.cast(pose, self.rng)
        dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                          ('ring', '<u2'), ('padding', '<u2')])
        points = np.empty(len(scan.rings), dtype=dtype)
        points['x'] = scan.sensor_points[:, 0]
        points['y'] = scan.sensor_points[:, 1]
        points['z'] = scan.sensor_points[:, 2]
        points['ring'] = scan.rings
        points['padding'] = 0

        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = 'sim_vlp16'
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name=name, offset=offset, datatype=datatype, count=1)
            for name, offset, datatype in (
                ('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
                ('z', 8, PointField.FLOAT32), ('ring', 12, PointField.UINT16))]
        cloud.is_bigendian = False
        cloud.point_step = dtype.itemsize
        cloud.row_step = len(points) * dtype.itemsize
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = None
    try:
        node = TerrainPointcloudGenerator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

