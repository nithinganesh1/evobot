#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
import pyaudio
import json
import os
model_path = os.path.join(os.path.dirname(__file__), "vosk-model-small-en-us-0.15")


class MicManagerNode(Node):
    def __init__(self):
        super().__init__('mic_manager_node')

        # Publisher for recognized speech
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)

        # Load Vosk model (make sure the path exists)
        model_path = os.path.join(os.path.dirname(__file__), "vosk-model-small-en-us-0.15")
        if not os.path.exists(model_path):
            self.get_logger().error(f"Vosk model not found at {model_path}")
            exit(1)

        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)

        # Setup microphone stream
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=8000
        )
        self.stream.start_stream()

        # Timer to process mic input
        self.timer = self.create_timer(0.1, self.process_audio)

        self.get_logger().info("🎤 Mic Manager Node started. Listening...")

    def process_audio(self):
        """Reads mic audio and publishes recognized text."""
        data = self.stream.read(4000, exception_on_overflow=False)
        if self.recognizer.AcceptWaveform(data):
            result = json.loads(self.recognizer.Result())
            text = result.get("text", "").strip()
            if text:
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
                self.get_logger().info(f"🗣️ Recognized: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = MicManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
