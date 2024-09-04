import numpy as np

import rclpy
from rclpy.node import Node
import pdb
import wave
import time

from array import array

from audio_utils_msgs.msg import AudioFrame
from odas_ros_msgs.msg import OdasSst, OdasSstArrayStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

def nbits_to_format(nbits):
    if nbits == 8:
        return 'signed_8'
    elif nbits == 16:
        return 'signed_16'
    elif nbits == 32:
        return 'signed_32'
    else:
        raise ValueError('Not supported format (nbits={})'.format(nbits))


class AudioTrackNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_track_node")

        self.declare_parameters("", [
            ("format", 2),
            ("channels", 4),
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


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.audio_pub_vf = self.create_publisher(AudioFrame, 'voice_fragment', 1)

        self.format = nbits_to_format(self.format*8)

        self.audio_sources = {}
        self.time_limit = 5
        self.silence_limit = 2
        
        
        self.audio_sub_sss = self.create_subscription(
            AudioFrame, "sss", self.get_sss ,qos_profile)
            
            
        self.audio_sub_sst = self.create_subscription(
            OdasSstArrayStamped, "sst", self.get_sst ,qos_profile)
        
        #self.all_audio = np.array([])
        #self.all_audio_time = 0
        self.get_logger().info("AudioTrack node started")
        #self.wv = open('raw', 'wb')


    def to_whisper(self, data, sample_count):
    
        audio_frame_msg = AudioFrame()
        audio_frame_msg.header.stamp = self.get_clock().now().to_msg()
        audio_frame_msg.header.frame_id = self.frame_id
        audio_frame_msg.format = self.format
        audio_frame_msg.channel_count = 1
        audio_frame_msg.sampling_frequency = self.rate
        audio_frame_msg.frame_sample_count = sample_count*self.chunk
        audio_frame_msg.data = data
        
        self.get_logger().info("Sending to whisper")

        self.audio_pub_vf.publish(audio_frame_msg)
        #self.wv.write(data)

        


    def get_sst(self, msg: OdasSstArrayStamped):
    
        for source_idx,source in enumerate(msg.sources):
        
            if source.activity >= 0.9 or source.id in self.audio_sources.keys():
                if source.id not in self.audio_sources.keys():
                    self.audio_sources[source.id] = {"silence_time": 0.0, "data": array('B',[]), "total_time": 0.0, "channel": source_idx, "sample_count": 0}
                    
                self.audio_sources[source.id]["channel"] = source_idx
                
                if source.activity >= 0.9:
                    self.audio_sources[source.id]["silence_time"] = 0.0
                    #self.get_logger().info("Activity registered")
                else:
                    self.audio_sources[source.id]["silence_time"] += self.chunk/self.rate
                    
                self.audio_sources[source.id]["total_time"] += self.chunk/self.rate

    def get_sss(self, msg: AudioFrame):
        if (msg.format != self.format or
            msg.channel_count != self.channels or
            msg.sampling_frequency != self.rate or
            msg.frame_sample_count != self.chunk):
            self.get_logger().error(
                'Invalid frame (msg.format={}, msg.channel_count={}, msg.sampling_frequency={}, msg.frame_sample_count={})'

               .format(msg.format, msg.channel_count, msg.sampling_frequency, msg.frame_sample_count))
            return

        #self.all_audio = np.concatenate((self.all_audio,np.array(msg.data).view(np.int16)),axis=0)
        #self.all_audio = np.concatenate((self.all_audio,np.array(msg.data)),axis=0)
        #pdb.set_trace()
        #data = array('B',np.array(msg.data).view(np.int16)[0::self.channels].copy().view(np.uint8).tolist())
        #self.wv.write(data)
        to_delete = []
        for aus in self.audio_sources.keys():
            #print(aus, self.audio_sources[aus]["total_time"])
            self.audio_sources[aus]["data"].extend(np.array(msg.data).view(np.int16)[self.audio_sources[aus]["channel"]::self.channels].copy().view(np.uint8).tolist()) #np.concatenate((self.audio_sources[aus]["data"], np.array(msg.data).view(np.int16)[self.audio_sources[aus]["channel"]::self.channels]), axis=0)

            self.audio_sources[aus]["sample_count"] += 1

            if self.audio_sources[aus]["silence_time"] >= self.silence_limit: 

                if self.audio_sources[aus]["total_time"] > self.audio_sources[aus]["silence_time"]:
                    self.to_whisper(self.audio_sources[aus]["data"], self.audio_sources[aus]["sample_count"])
                    self.get_logger().info("Deleting")
                
                to_delete.append(aus)                
                    
            elif self.audio_sources[aus]["total_time"] >= self.time_limit:
                self.to_whisper(self.audio_sources[aus]["data"], self.audio_sources[aus]["sample_count"])
                self.audio_sources[aus]["data"] = array('B',[])
                self.audio_sources[aus]["total_time"] = 0.0
                self.audio_sources[aus]["sample_count"] = 0
        
        for aus in to_delete:
            del self.audio_sources[aus]

        
        #if self.all_audio_time >= 5:
        #    #wv.write(self.all_audio)
        #    #wv.close()
        #    self.all_audio = np.array([])
        #    self.all_audio_time = 0
        
        #self.all_audio_time += self.chunk/self.rate


def main(args=None):
    rclpy.init(args=args)
    node = AudioTrackNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
