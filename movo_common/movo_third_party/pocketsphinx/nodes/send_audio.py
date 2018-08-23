#!/usr/bin/python

from time import sleep
import re

import pyaudio
import wave
import subprocess

import rospy

from std_msgs.msg import String


class AudioMessage(object):
    """Class to publish audio to topic"""

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("sphinx_audio", String, queue_size=10)

        # initialize node
        rospy.init_node("audio_control")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        # All set. Publish to topic
        self.transfer_audio_msg()

    def transfer_audio_msg(self):
        """Function to publish input audio to topic"""

        rospy.loginfo("audio input node will start after delay of 2 seconds")
        sleep(2)

        p=pyaudio.PyAudio()

        # Params
        self._input = "~input"
        _rate_bool = False

        # Find microphone index
        mic_name = rospy.get_param(self._input)
        microphone_plughw_index = None
        for i in range(p.get_device_count()):
            dev = p.get_device_info_by_index(i)
            if mic_name in dev["name"] :
                microphone_plughw_index = re.search("hw:(.*),", dev["name"]).group(1)
        if microphone_plughw_index is None:
            rospy.logerr("Microphone could not be found.")
            return
        rospy.loginfo("Microphone %s is : %s" % (mic_name, microphone_plughw_index))

        # This is the required audio input config (required by Pocketsphinx)
        FORMAT = pyaudio.paInt16
        N_CHANNELS = 1
        RATE = 16000
        BUFSIZE = 4096

        # Checking if audio file given or system microphone is needed
        if rospy.has_param(self._input):
            cmd = 'arecord -D plughw:%s -r %d -f S16_LE -c %d' % (microphone_plughw_index, RATE, N_CHANNELS)
            print ("COMMAND IS : {}".format(cmd))
            record_process = subprocess.Popen(cmd.split(' '), stdout = subprocess.PIPE, stderr = subprocess.PIPE)
            wav_file = wave.open(record_process.stdout, 'rb')
        else:
            rospy.logerr("No input means provided. Please use the launch file instead")

        while not rospy.is_shutdown():
            buf = String("")
            buf.data = wav_file.readframes(BUFSIZE)
            if buf.data:
                # Publish audio to topic
                self.pub_.publish(buf)
                if _rate_bool:
                    rate.sleep()
            else:
                rospy.loginfo("Buffer returned null")
                break

        record_process.terminate()
        wav_file.close()
        p.terminate()

    @staticmethod
    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed

if __name__ == "__main__":
    AudioMessage()
