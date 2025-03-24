#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from robot_common_interfaces.msg import SerComStruct

# This ROS 2 node, `PwmSubNode`, subscribes to the `motor_throttle_control` topic, 
# receiving messages that dictate PWM values and control signals for a differential drive robot's motors.
# It sets the duty cycles for left and right motors based on incoming messages,
# manages reverse direction states, and controls gear settings using the Raspberry Pi's GPIO pins.

class PwmSubNode(Node):
    def __init__(self):
        super().__init__('ser_com_subscriber')
        self.get_logger().info("Node initiated")
        self.sub_ = self.create_subscription(SerComStruct, 'motor_throttle_control', self.listener_callback, 20)

        # Pin layout
        self.pwm_l = PWMOutputDevice(18, frequency=500)  # PWM for left wheel
        self.pwm_r = PWMOutputDevice(13, frequency=500)  # PWM for right wheel
        self.is_reverse_l = DigitalOutputDevice(15)      # Reverse state for left wheel
        self.is_reverse_r = DigitalOutputDevice(19)      # Reverse state for right wheel
        self.gear_l = DigitalOutputDevice(14)            # Gear state for left wheel
        self.gear_r = DigitalOutputDevice(26)            # Gear state for right wheel

        self.prev_msg = None  # To track gear state changes

    def listener_callback(self, msg):
        buff = 1.0
        self.is_reverse_l.value =  msg.is_reverse_dir_l
        self.is_reverse_r.value =  msg.is_reverse_dir_r

        # Set PWM values (scale to [0, 1] for gpiozero)
        if self.is_reverse_l.value:
            self.pwm_l.value = min( buff *abs(msg.pwm_l/1000) ,0.53)#max(0.0, min(msg.pwm_l / 100.0, 1.0))  # Scale from 0-100 to 0-1
        else:
            self.pwm_l.value = min( buff * abs(msg.pwm_l/1000) ,0.53)

        if self.is_reverse_r.value:
            self.pwm_r.value = min( buff * abs(msg.pwm_r/1000) ,0.53)#max(0.0, min(msg.pwm_l / 100.0, 1.0))  # Scale from 0-100 to 0-1
        else:
            self.pwm_r.value = min( buff * abs(msg.pwm_r/1000) ,0.53)

        self.get_logger().info(f"Sending negative PWM: \nLeft: {self.pwm_l.value}, Right: {self.pwm_r.value}\n dir_L:{self.is_reverse_l.value},dir_R{self.is_reverse_r.value}")

        # Set gear values
        self.gear_l.value = msg.gear
        self.gear_r.value = msg.gear
        
        if self.prev_msg != msg.gear:
            self.get_logger().info(f"Gear changed to {'LOW' if msg.gear else 'HIGH'}")
            self.prev_msg = msg.gear


def main(args=None):
    rclpy.init(args=None)
    node = PwmSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
