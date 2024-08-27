import numpy as np
from scipy.fft import rfft, rfftfreq

import rclpy
from rclpy.node import Node
import pdb

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


class AudioFFTNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_fft_node")

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

        self.audio_sub = self.create_subscription(
            AudioFrame, "raw", self.get_fft ,1)

        self.audio_frames = np.array([])
        self.count = 0
        self.num_samples = int(self.rate/self.chunk)

        self.format = nbits_to_format(self.format*8)

        self.get_logger().info("AudioFFT node started")

    def get_fft(self, msg: AudioFrame):
        if (msg.format != self.format or
            msg.channel_count != self.channels or
            msg.sampling_frequency != self.rate or
            msg.frame_sample_count != self.chunk):
            self.get_logger().error(
                'Invalid frame (msg.format={}, msg.channel_count={}, msg.sampling_frequency={}, msg.frame_sample_count={})'
                .format(msg.format, msg.channel_count, msg.sampling_frequency, msg.frame_sample_count))
            return

        self.audio_frames = np.concatenate((self.audio_frames, np.array(msg.data).view(np.int16)[0::6]), axis=0)

        self.count += 1

        if self.count >= self.num_samples: 
            yf = rfft(self.audio_frames) 
            xf = rfftfreq(self.chunk*self.count, 1/self.rate) 
            freqs = xf[np.argpartition(np.abs(yf),-3)[-3:]].tolist()
            self.get_logger().info('Freq: %.2f' % freqs[-1]) 
            self.count = 0 
            self.audio_frames = np.array([])

def main(args=None):
    rclpy.init(args=args)
    node = AudioFFTNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
