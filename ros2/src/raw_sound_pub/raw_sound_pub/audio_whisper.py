import numpy as np
import torch
from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline

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


class AudioWhisperNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_whisper_node")

        self.declare_parameters("", [
            ("format", 2),
            ("channels", 1),
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



        self.format = nbits_to_format(self.format*8)
        
        device = "cuda:0" if torch.cuda.is_available() else "cpu"
        torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32

        print(device)

        model_id = "openai/whisper-large-v3"

        model = AutoModelForSpeechSeq2Seq.from_pretrained(
            model_id, torch_dtype=torch_dtype, low_cpu_mem_usage=True, use_safetensors=True
        )
        model.to(device)

        processor = AutoProcessor.from_pretrained(model_id)
        
        self.pipe = pipeline(
            "automatic-speech-recognition",
            model=model,
            tokenizer=processor.tokenizer,
            feature_extractor=processor.feature_extractor,
            torch_dtype=torch_dtype,
            device=device,
        )

            
        self.audio_sub_vf = self.create_subscription(
            AudioFrame, "voice_fragment", self.whisper ,10)
            
        
        self.get_logger().info("AudioWhisper node started")


        
    def whisper(self, msg: AudioFrame):

        sample = np.array(msg.data).view(np.int16).astype(np.float32) / 32768.0

        result = self.pipe(sample)
        self.get_logger().info(result["text"])


def main(args=None):
    rclpy.init(args=args)
    node = AudioWhisperNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
