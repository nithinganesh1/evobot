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

        # -------- SPEED SCALES --------
        self.base_speed = 0.30       # Overall speed (30%)
        self.linear_scale = 1.0
        self.angular_scale = 1.0

        self.step = 0.05             # 5% step
        self.max_scale = 1.0
        self.min_scale = 0.0
        self.deadzone = 0.10


        self.last_buttons = []
        self.wait_for_zero = True

        self.get_logger().info("Evobot Joystick Controller Started")

    def joy_callback(self, msg):
        # -------- AXIS MAP --------
        AXIS_LX = 0   # Left stick horizontal
        AXIS_LY = 1   # Left stick vertical

        # -------- BUTTON MAP (Xbox) --------
        BTN_A = 0
        BTN_B = 1
        BTN_X = 3
        BTN_Y = 4
        BTN_LB = 6
        BTN_RB = 7

        # -------- JOYSTICK INPUT --------
        lx = msg.axes[AXIS_LX]
        ly = msg.axes[AXIS_LY]

        if abs(lx) < self.deadzone:
            lx = 0.0
        if abs(ly) < self.deadzone:
            ly = 0.0

        # Wait until both lx and ly are zero at the same time before publishing
        if self.wait_for_zero:
            if lx == 0.0 and ly == 0.0:
                self.get_logger().info("Joystick centered (lx=0, ly=0). Starting to publish cmd_vel.")
                self.wait_for_zero = False
            else:
                # Still waiting for both axes to be zero
                self.last_buttons = msg.buttons
                return

        # -------- OVERALL SPEED --------
        if msg.buttons[BTN_RB] and not self.last_buttons[BTN_RB]:
            self.base_speed = min(self.base_speed + self.step, 1.0)
            self.get_logger().info(f"Base speed: {int(self.base_speed * 100)}%")

        if msg.buttons[BTN_LB] and not self.last_buttons[BTN_LB]:
            self.base_speed = max(self.base_speed - self.step, 0.0)
            self.get_logger().info(f"Base speed: {int(self.base_speed * 100)}%")

        # -------- LINEAR SCALE --------
        if msg.buttons[BTN_Y] and not self.last_buttons[BTN_Y]:
            self.linear_scale = min(self.linear_scale + self.step, self.max_scale)
            self.get_logger().info(f"Linear scale: {int(self.linear_scale * 100)}%")

        if msg.buttons[BTN_A] and not self.last_buttons[BTN_A]:
            self.linear_scale = max(self.linear_scale - self.step, self.min_scale)
            self.get_logger().info(f"Linear scale: {int(self.linear_scale * 100)}%")

        # -------- ANGULAR SCALE --------
        if msg.buttons[BTN_B] and not self.last_buttons[BTN_B]:
            self.angular_scale = min(self.angular_scale + self.step, self.max_scale)
            self.get_logger().info(f"Angular scale: {int(self.angular_scale * 100)}%")

        if msg.buttons[BTN_X] and not self.last_buttons[BTN_X]:
            self.angular_scale = max(self.angular_scale - self.step, self.min_scale)
            self.get_logger().info(f"Angular scale: {int(self.angular_scale * 100)}%")

        # -------- DIRECT DRIVE --------
        twist = Twist()
        twist.linear.x = ly * self.base_speed * self.linear_scale
        twist.angular.z = lx * self.base_speed * self.angular_scale

        self.pub.publish(twist)
        self.last_buttons = msg.buttons
        twist.linear.x = ly * self.base_speed * self.linear_scale
        twist.angular.z = lx * self.base_speed * self.angular_scale

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
