from flask import Flask, render_template, request, jsonify
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

# ---------------- Flask App ----------------
app = Flask(__name__)

# Default speed multipliers
speed_linear = 0.5
speed_angular = 0.5

# ---------------- ROS2 Node ----------------
class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lock = threading.Lock()
        self.current_twist = Twist()
        self.pressed = False
        # Timer to continuously publish Twist at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_twist)

    def set_pressed(self, linear, angular, pressed):
        """Set the current movement state"""
        with self.lock:
            self.current_twist.linear.x = linear
            self.current_twist.angular.z = angular
            self.pressed = pressed

    def publish_twist(self):
        with self.lock:
            if self.pressed:
                self.publisher.publish(self.current_twist)
            # else: do nothing, don't send zero unnecessarily

# ---------------- Initialize ROS Node ----------------
def start_ros_node():
    rclpy.init()
    node = WebControlNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return node

ros_node = start_ros_node()

# ---------------- Flask Routes ----------------
@app.route('/')
def index():
    return render_template('buttons.html')  # buttons.html must be in templates/

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

    # Update ROS node pressed state
    ros_node.set_pressed(linear, angular, pressed)
    return jsonify({'status': 'ok'})

@app.route('/set_speed', methods=['POST'])
def set_speed():
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
