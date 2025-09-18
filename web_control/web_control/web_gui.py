from flask import Flask, render_template, request, jsonify
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

app = Flask(__name__)

# Default speed multipliers
speed_linear = 0.5
speed_angular = 0.5

# ROS2 publisher node
class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_twist = Twist()
        self.lock = threading.Lock()
        # Timer to continuously publish current Twist
        self.timer = self.create_timer(0.05, self.publish_twist)

    def set_twist(self, linear, angular):
        with self.lock:
            self.current_twist.linear.x = linear
            self.current_twist.angular.z = angular

    def stop(self):
        with self.lock:
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0

    def publish_twist(self):
        with self.lock:
            self.publisher.publish(self.current_twist)

# Initialize ROS node
def start_ros_node():
    rclpy.init()
    node = WebControlNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return node

# Create global ROS node instance
ros_node = start_ros_node()

# ---------------- Flask routes ----------------
@app.route('/')
def index():
    return render_template('buttons.html')  # Make sure this file is in templates/

@app.route('/control', methods=['POST'])
def control():
    data = request.json
    direction = data.get('direction')
    pressed = data.get('pressed', False)

    linear = 0.0
    angular = 0.0

    if pressed:
        if direction == 'forward':
            linear = speed_linear
        elif direction == 'backward':
            linear = -speed_linear
        elif direction == 'left':
            angular = speed_angular
        elif direction == 'right':
            angular = -speed_angular

        ros_node.set_twist(linear, angular)
    else:
        ros_node.stop()

    return jsonify({'status': 'ok'})

@app.route('/set_speed', methods=['POST'])
def set_speed():
    """Optional: adjust speed sliders from UI"""
    global speed_linear, speed_angular
    data = request.json
    speed_linear = float(data.get('linear', speed_linear))
    speed_angular = float(data.get('angular', speed_angular))
    return jsonify({'status': 'ok', 'linear': speed_linear, 'angular': speed_angular})

# ---------------- Main ----------------
def main():
    app.run(host='0.0.0.0', port=5000, debug=True)

if __name__ == '__main__':
    main()
