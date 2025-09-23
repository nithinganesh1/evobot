#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import wave
from piper import PiperVoice

# Load the Piper voice model
MODEL_DIR = os.path.join(os.path.dirname(__file__), "en_US-amy-medium.onnx")
voice = PiperVoice.load(MODEL_DIR)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "tts_output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        # Subscribe to GPT responses
        self.subscription = self.create_subscription(
            String,
            'reply_text',
            self.listener_callback,
            10
        )

        self.get_logger().info("🔊 TTS Node started, waiting for replies...")

    def listener_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"💬 TTS received: {text}")

        # Output file path
        output_file = os.path.join(OUTPUT_DIR, "response.wav")

        # Generate audio using Piper
        try:
            with wave.open(output_file, "wb") as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)  # 16-bit audio
                wav_file.setframerate(24000)  # Piper default sample rate
                voice.synthesize_wav(text, wav_file)

            self.get_logger().info(f"✅ Audio saved to {output_file}")

            # Optionally play the audio immediately
            import simpleaudio as sa
            wave_obj = sa.WaveObject.from_wave_file(output_file)
            play_obj = wave_obj.play()
            play_obj.wait_done()

        except Exception as e:
            self.get_logger().error(f"Error generating TTS: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
