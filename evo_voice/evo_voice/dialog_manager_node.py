#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
from evo_voice.key import api_key

openai.api_key = api_key

CONTEXT = "You are a helpful assistant for a robot."

def get_response(question):
    try:
        response = openai.ChatCompletion.create(
            model="gpt-4.1-nano",  
            messages=[
                {"role": "system", "content": CONTEXT},
                {"role": "user", "content": question}
            ],
            max_tokens=100,
            temperature=0.7
        )
        answer = response.choices[0].message["content"].strip()
        return answer if answer else "I don't know the answer."
    except Exception as e:
        print(f"Error: {e}")
        return "I don't know the answer."

class DialogManagerNode(Node):
    def __init__(self):
        super().__init__('dialog_manager_node')

        # Subscribe to STT results
        self.subscription = self.create_subscription(
            String,
            'speech_text',
            self.listener_callback,
            10
        )

        # Publisher for GPT replies
        self.publisher_ = self.create_publisher(String, 'reply_text', 10)

        self.get_logger().info("🤖 Dialog Manager Node started. Waiting for speech...")

    def listener_callback(self, msg):
        question = msg.data.strip()
        if not question:
            return
        
        self.get_logger().info(f"👂 Heard: {question}")

        # Call GPT
        answer = get_response(question)

        # Publish GPT answer
        reply_msg = String()
        reply_msg.data = answer
        self.publisher_.publish(reply_msg)

        self.get_logger().info(f"💬 Response: {answer}")

def main(args=None):
    rclpy.init(args=args)
    node = DialogManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
