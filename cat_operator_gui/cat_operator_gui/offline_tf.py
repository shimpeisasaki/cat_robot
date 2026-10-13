"""Replay odometry/sensor transforms, excluding recorded SLAM/localization transforms."""


def keep_transform(transform):
    parent = transform.header.frame_id.lstrip('/')
    child = transform.child_frame_id.lstrip('/')
    return parent != 'map' and child != 'map'


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    rclpy.init()

    class Relay(Node):
        def __init__(self):
            super().__init__('offline_tf_filter')
            dynamic = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
            static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.static_transforms = {}
            self.tf_pub = self.create_publisher(TFMessage, '/tf', dynamic)
            self.static_pub = self.create_publisher(TFMessage, '/tf_static', static)
            self.create_subscription(TFMessage, '/offline/raw_tf', self.dynamic, dynamic)
            self.create_subscription(TFMessage, '/offline/raw_tf_static', self.static, static)

        def dynamic(self, message):
            transforms = [item for item in message.transforms if keep_transform(item)]
            if transforms:
                self.tf_pub.publish(TFMessage(transforms=transforms))

        def static(self, message):
            for item in message.transforms:
                if keep_transform(item):
                    self.static_transforms[item.child_frame_id] = item
            if self.static_transforms:
                self.static_pub.publish(TFMessage(transforms=list(self.static_transforms.values())))

    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
