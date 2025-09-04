#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32

def qos_from_params(node: Node) -> QoSProfile:
    node.declare_parameter('qos.reliability', 'reliable')       # 'reliable'|'best_effort'
    node.declare_parameter('qos.durability', 'volatile')        # 'volatile'|'transient_local'
    node.declare_parameter('qos.depth', 10)
    rel = node.get_parameter('qos.reliability').get_parameter_value().string_value
    dur = node.get_parameter('qos.durability').get_parameter_value().string_value
    depth = node.get_parameter('qos.depth').get_parameter_value().integer_value

    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if rel == 'reliable'
                    else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if dur == 'transient_local'
                   else DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth
    )

class FilterSubscriber(Node):
    def __init__(self):
        super().__init__('filter_subscriber')
        self.declare_parameter('threshold', 50)

        qos = qos_from_params(self)
        self.sub = self.create_subscription(Int32, 'numbers', self._cb, qos)

    def _cb(self, msg: Int32):
        threshold = self.get_parameter('threshold').get_parameter_value().integer_value
        if msg.data >= threshold:
            self.get_logger().info(f'Filtered IN: {msg.data}')
        else:
            # Keep this quiet to reduce spam; uncomment to see all flow:
            # self.get_logger().debug(f'Filtered OUT: {msg.data}')
            pass

def main():
    rclpy.init()
    node = FilterSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()