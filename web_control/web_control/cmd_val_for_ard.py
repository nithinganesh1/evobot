#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

arduino_port = "/dev/ttyUSB0"
baudrate = 9600
ser = serial.Serial(arduino_port, baudrate, timeout=1)
time.sleep(2)

button_pressed = False


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('raspi_cmd_vel')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.create_timer(0.1, self.send_safe_signal)
        self.last_cmd_time = 0

    def cmd_vel_callback(self, msg):
        global button_pressed

        linear = msg.linear.x
        angular = msg.angular.z

        # Deadzone: treat very small values as stop
        if abs(linear) < 0.05 and abs(angular) < 0.05:
            button_pressed = False
            ser.write(b"S\n")
            return

        button_pressed = True
        self.last_cmd_time = time.time()

        # Differential drive
        left = linear - angular
        right = linear + angular

        # Clip to [-1, 1]
        left = max(min(left, 1.0), -1.0)
        right = max(min(right, 1.0), -1.0)

        # Send as floats to Arduino
        cmd = f"L{left:.2f}R{right:.2f}\n"
        ser.write(cmd.encode('utf-8'))

        self.get_logger().info(
            f"Linear: {linear:.2f}, Angular: {angular:.2f} | Left: {left:.2f}, Right: {right:.2f}"
        )

    def send_safe_signal(self):
        if not button_pressed or (time.time() - self.last_cmd_time > 0.2):
            ser.write(b"S\n")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CmdVelPublisher node")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ser.close()


if __name__ == '__main__':
    main()
