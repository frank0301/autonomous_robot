#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_muxed', 30)
        self.sub_joy = self.create_subscription(Twist, '/cmd_vel_joy', self.joy_callback, 30)
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel', self.nav_callback, 30)
        self.last_source = 'nav'

    def joy_callback(self, msg):
        self.last_source = 'joy'
        self.publisher.publish(msg)

    def nav_callback(self, msg):
        if self.last_source != 'joy':
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
