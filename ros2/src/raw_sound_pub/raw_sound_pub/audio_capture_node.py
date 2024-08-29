import pyaudio

import rclpy
from rclpy.node import Node

from audio_utils_msgs.msg import AudioFrame

def nbits_to_format(nbits):
    if nbits == 8:
        return 'signed_8'
    elif nbits == 16:
        return 'signed_16'
    elif nbits == 32:
        return 'signed_32'
    else:
        raise ValueError('Not supported format (nbits={})'.format(nbits))

class AudioCapturerNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_capturer_node")

        self.declare_parameters("", [
            ("format", 2),
            ("channels", 6),
            ("rate", 16000),
            ("chunk", 128),
            ("device", 2),
            ("frame_id", "odas")
        ])

        self.format = self.get_parameter(
            "format").get_parameter_value().integer_value
        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value
        self.rate = self.get_parameter(
            "rate").get_parameter_value().integer_value
        self.chunk = self.get_parameter(
            "chunk").get_parameter_value().integer_value
        device = self.get_parameter(
            "device").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        if device < 0:
            device = None

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.audio.get_format_from_width(self.format),
            channels=self.channels,
            rate=self.rate,
            input=True,
            input_device_index=device
        )

        self.audio_pub = self.create_publisher(
            AudioFrame, "raw", 1)

        self.get_logger().info("AudioCapturer node started")

    def destroy_node(self) -> bool:
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        return super().destroy_node()

    def work(self) -> None:

        while rclpy.ok:
            data = self.stream.read(self.chunk, exception_on_overflow=False)

            audio_frame_msg = AudioFrame()
            audio_frame_msg.header.stamp = self.get_clock().now().to_msg()
            audio_frame_msg.header.frame_id = self.frame_id
            audio_frame_msg.format = nbits_to_format(self.format*8)
            audio_frame_msg.channel_count = self.channels
            audio_frame_msg.sampling_frequency = self.rate
            audio_frame_msg.frame_sample_count = self.chunk
            audio_frame_msg.data = data


            self.audio_pub.publish(audio_frame_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapturerNode()
    node.work()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
