#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_common_interfaces.msg import KeyCtrl
from gpiozero import PWMOutputDevice, DigitalOutputDevice

class CmdVelToPwmGPIO(Node):
    LINEAR_SCALE = 1.0  # 线速度缩放
    ANGULAR_SCALE = 1.4 # 角速度缩放
    MAX_SPEED = 0.49

    manualCtrl = True
    def __init__(self):
        super().__init__('cmd_vel_to_pwm_gpio')
        self.subscription = self.create_subscription(KeyCtrl, '/cmd_joy', self.cmd_joy_callback, 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # 机器人参数
        self.wheel_base = 0.475  # 轮距 (米)
        # GPIO 设置
        try:
            self.pwm_l = PWMOutputDevice(18, frequency=500)
            self.pwm_r = PWMOutputDevice(13, frequency=500)
            self.dir_l = DigitalOutputDevice(15)
            self.dir_r = DigitalOutputDevice(19)
            self.gear_l = DigitalOutputDevice(14)
            self.gear_r = DigitalOutputDevice(26)
        except Exception as e:
            self.get_logger().error(f"GPIO 初始化失败: {e}")
            return

    def cmd_joy_callback(self, msg):
        self.manualCtrl = not msg.allow_nav
        if self.manualCtrl:
            """ 处理 cmd_vel 并控制 GPIO """
            linear_x = msg.manual_spd.linear.x * self.LINEAR_SCALE
            angular_z = msg.manual_spd.angular.z * self.ANGULAR_SCALE

            v_left = linear_x - (self.wheel_base / 2.0) * angular_z
            v_right = linear_x + (self.wheel_base / 2.0) * angular_z

            v_left = v_left if v_left >= 0 else 0
            v_right = v_right if v_right >= 0 else 0

            self.get_logger().info(f"v_left: {v_left}, v_right: {v_right} | mode:{self.manualCtrl}")

            # 速度转换为 PWM
            pwm_left = self.velocity_to_pwm(v_left)
            pwm_right = self.velocity_to_pwm(v_right)

            # 设置方向
            self.dir_l.value = pwm_left < 0
            self.dir_r.value = pwm_right < 0

            # 设置 PWM 取值范围 0~1
            self.pwm_l.value = min(abs(pwm_left), self.MAX_SPEED)
            self.pwm_r.value = min(abs(pwm_right), self.MAX_SPEED)


            self.gear_l.value = False
            self.gear_r.value = False
        self.get_logger().info(f"PWM -> Left: {(-1 if self.dir_l.value else 1) * self.pwm_l.value}, Right: {(-1 if self.dir_r.value else 1)*self.pwm_r.value} | mode:{self.manualCtrl}")
    def cmd_vel_callback(self,msg):
        if not self.manualCtrl:
            linear_x = msg.linear.x * self.LINEAR_SCALE
            angular_z = msg.angular.z * self.ANGULAR_SCALE

            v_left = linear_x - (self.wheel_base / 2.0) * angular_z
            v_right = linear_x + (self.wheel_base / 2.0) * angular_z

            v_left = v_left if v_left >= 0 else 0
            v_right = v_right if v_right >= 0 else 0

            pwm_left = self.velocity_to_pwm(v_left)
            pwm_right = self.velocity_to_pwm(v_right)

            
            self.dir_l.value = pwm_left < 0
            self.dir_r.value = pwm_right < 0

            
            self.pwm_l.value = min(abs(pwm_left), self.MAX_SPEED)
            self.pwm_r.value = min(abs(pwm_right), self.MAX_SPEED)

            self.get_logger().info(f"PWM -> Left: {self.pwm_l.value}, Right: {self.pwm_r.value}|mode{self.manualCtrl}")

            self.gear_l.value = False
            self.gear_r.value = False
            
    def velocity_to_pwm(self, velocity):
        """ 将轮速转换为 PWM 值 """
        threshold = 0.05
        if abs(velocity) < threshold:
            return 0  # 速度太小，停止
        pwm_value = (0.50 * abs(velocity) + 0.37)
        if velocity < 0:
            return -pwm_value 
        else:
            return pwm_value

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPwmGPIO()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
