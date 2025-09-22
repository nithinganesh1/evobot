#!/bin/python3
import rclpy
from rclpy.node import Node
from std.msgs import Bool
from vosk import Model, KaldiRecognizer
import pyaudio
import json

class WakeWordNode(Node):
    def __init__(self):
        super().__init__('wake_word_node')
        self.publisher_ = self.create_publisher(Bool, 'wake_word_detected', 10)
        self.wake_word = "hey evo"
        self.get_logger().info('Wake word node started, listening for wake word...')

        # Vosk model setup
        self.model = Model('vosk-model-small-en-us-0.15')
        self.recognizer = KaldiRecognizer(self.model, 16000)


        # Audio stream setup
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=pyaudio.paInt16,
                                        channels=1,rate=16000,
                                        input=True,
                                        frames_per_buffer=8000)
        self.stream.start_stream()

        self.timer = self.create_timer(0.1, self.listen_for_wake_word)

        def listen_for_wake_word(self):
            data = self.stream.read(4000, exception_on_overflow = False)
            if self.recognizer.AcceptWaveform(data):
                result = self.recognizer.Result()
                text = result.get('text', '').lower()
                if self.wake_word in text:
                    msg = Bool()
                    msg.data = True
                    self.publisher_.publish(msg)
                    self.get_logger().info('Wake word detected!')

def main(args=None):
    rclpy.init(args=args)
    wake_word_node = WakeWordNode()
    rclpy.spin(wake_word_node)
    wake_word_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()