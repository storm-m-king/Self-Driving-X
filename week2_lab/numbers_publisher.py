#!/usr/bin/env python3
import random

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

class NumbersPublisher(Node):
    def __init__(self):
        super().__init__('numbers_publisher')
        self.declare_parameter('frequency', 5.0)  # Hz

        qos = qos_from_params(self)
        self.pub = self.create_publisher(Int32, 'numbers', qos)

        freq = max(0.001, self.get_parameter('frequency').get_parameter_value().double_value)
        self.timer = self.create_timer(1.0 / freq, self._tick)

    def _tick(self):
        msg = Int32()
        msg.data = random.randint(0, 100)
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main():
    rclpy.init()
    node = NumbersPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
