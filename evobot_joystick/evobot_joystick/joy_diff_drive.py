#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyDiffDrive(Node):

    def __init__(self):
        super().__init__('joy_diff_drive')

        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.speed = 0.30          # 30% initial speed
        self.speed_step = 0.05     # 5% step
        self.max_speed = 1.0
        self.min_speed = 0.0
        self.deadzone = 0.10

        self.last_buttons = []

        self.get_logger().info("Evobot Joystick Controller Started")

    def joy_callback(self, msg):
        if not self.last_buttons:
            self.last_buttons = msg.buttons
            return

        twist = Twist()

        # -------- AXIS MAP (Xbox default) --------
        AXIS_LX = 0   # Left stick horizontal
        AXIS_LY = 1   # Left stick vertical

        # -------- BUTTON MAP --------
        BTN_LB = 4
        BTN_RB = 5

        # -------- SPEED CONTROL --------
        if msg.buttons[BTN_RB] and not self.last_buttons[BTN_RB]:
            self.speed = min(self.speed + self.speed_step, self.max_speed)
            self.get_logger().info(f"Speed set to {int(self.speed * 100)}%")

        if msg.buttons[BTN_LB] and not self.last_buttons[BTN_LB]:
            self.speed = max(self.speed - self.speed_step, self.min_speed)
            self.get_logger().info(f"Speed set to {int(self.speed * 100)}%")

        # -------- JOYSTICK INPUT --------
        lx = msg.axes[AXIS_LX]
        ly = msg.axes[AXIS_LY]

        if abs(lx) < self.deadzone:
            lx = 0.0
        if abs(ly) < self.deadzone:
            ly = 0.0

        # Invert Y if needed
        # ly = -ly

        # -------- DIRECT DRIVE --------
        twist.linear.x = ly * self.speed
        twist.angular.z = lx * self.speed

        self.pub.publish(twist)
        self.last_buttons = msg.buttons


def main():
    rclpy.init()
    node = JoyDiffDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
