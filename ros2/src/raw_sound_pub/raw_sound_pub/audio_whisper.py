import numpy as np
import torch
from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline

import rclpy
from rclpy.node import Node
import pdb
import wave
import time
from groq import Groq
import os

from audio_utils_msgs.msg import AudioFrame
from pydub import AudioSegment

from raw_sound_pub_interfaces.msg import Text 

from gtts import gTTS


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
            generate_kwargs={"language": "en", "task": "transcribe"},
        )

            
        self.audio_sub_vf = self.create_subscription(
            AudioFrame, "voice_fragment", self.whisper ,10)

        self.audio_pub_text = self.create_publisher(Text, 'speech_txt', 10)
            
        self.audio_sub_text = self.create_subscription(
            Text, "speech_txt", self.txt2speech ,10)
            
        self.audio_pub_speech = self.create_publisher(AudioFrame, 'txt_speech', 10)

        self.get_logger().info("AudioWhisper node started")
        
        self.client = Groq(api_key=os.getenv("GROQ_API_KEY"))

        #self.wf = wave.open("raw.wav", 'wb')
        #self.wf.setnchannels(1)
        #self.wf.setsampwidth(2)
        #self.wf.setframerate(self.rate)

        
    def whisper(self, msg: AudioFrame):
    
        '''
        try:
            #wf.writeframes(np.array(msg.data).view(np.int16).tobytes())
            self.wf.writeframes(msg.data)
        except:
            pdb.set_trace()
        #wf.close()
        '''
        sample = np.array(msg.data).view(np.int16).astype(np.float32) / 32768.0
        
        time_before = time.time()
        result = self.pipe(sample)
        time_result = time.time()-time_before
        self.get_logger().info(f'%.2f %s' % (time_result,result["text"]))
        
        txt = Text()
        
        txt.data = result["text"]
        
        self.audio_pub_text.publish(txt)
        
    def txt2speech(self, msg: Text):
    
        #try:
        chat_completion = self.client.chat.completions.create(
            messages=[
            {
                "role": "user",
                "content": msg.data,
            }
        ],
            model="llama3-8b-8192"
        )

        #print(chat_completion.choices[0].message.content)
        
        response = chat_completion.choices[0].message.content
    
        self.get_logger().info(response)
    
        tts = gTTS(text=response, lang='en', slow=False)
        tts.save("test.mp3")
        sound = AudioSegment.from_mp3("test.mp3")
    
        audio_frame_msg = AudioFrame()
        audio_frame_msg.header.stamp = self.get_clock().now().to_msg()
        audio_frame_msg.header.frame_id = self.frame_id
        audio_frame_msg.format = nbits_to_format(sound.sample_width*8)
        audio_frame_msg.channel_count = sound.channels
        audio_frame_msg.sampling_frequency = sound.frame_rate
        audio_frame_msg.frame_sample_count = int(sound.frame_count())
        audio_frame_msg.data = sound._data
        
        self.get_logger().info("Sending text")
        self.audio_pub_speech.publish(audio_frame_msg)
        #except:
        #    print("problem")


def main(args=None):
    rclpy.init(args=args)
    node = AudioWhisperNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
