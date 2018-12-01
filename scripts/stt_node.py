#!/usr/bin/python3

import sys
import time
import getopt
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
from google.api_core.exceptions import Unknown as Unknown_except

from success_ros_msgs.msg import (
    Speech as Speech_msg 
)

import rospy
import threading
import actionlib
import signal

from std_srvs.srv import (
    SetBool,
    SetBoolResponse
)

from audio_streaming import (
    AudioStreamingObjectTopic,
    AudioStreamingObjectDirect
)


class AudioStreamingController():
    """ Main code that controls when the audio starts and ends 
    """
    def __init__(self):

        self._stt_pub = rospy.Publisher('stt', Speech_msg, queue_size=10)
        self._enable_stt_serv = rospy.Service('toggle_stt', SetBool, self._toggle_stt)
        self._enable_flag = False
        self._enable_event = threading.Event()
        self._audio_method_flag = rospy.get_param("google_stt/method", "mic")
        if self._audio_method_flag == 'topic':
            self._audio_method = AudioStreamingObjectTopic
        else:
            self._audio_method = AudioStreamingObjectDirect

    def _toggle_stt(self, req):
        self._enable_flag = req.data
        if self._enable_flag:
            self._enable_event.set()
        else:
            self._enable_event.clear()
        msg = "STT now {}".format("enabled" if self._enable_flag else "disabled")
        rospy.loginfo(msg)
        return SetBoolResponse(True, msg)

    def _close_loop(self, stream):
        """This is a 60 second timer that closes the stream in the end
        This is to get around Google's restriction of 1 min for continous recognition.
        """
        for i in range(0,120):
            time.sleep(0.5)
            if rospy.is_shutdown() or not self._enable_flag:
                break

        stream.close()

    def spin(self):
        while not rospy.is_shutdown():

            stream = self._audio_method()

            #check if STT is enabled
            if self._enable_flag:
                #start the stream if closed
                if stream.closed:
                    stream.start()
                #close thread
                self._close_thread = threading.Thread(target=self._close_loop, args=(stream,))
                self._close_thread.start()
                #start the recongition
                client = speech.SpeechClient()
                config = types.RecognitionConfig(
                    encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
                    sample_rate_hertz=16000,
                    language_code='en-US'
                )
                streaming_config = types.StreamingRecognitionConfig(
                    config=config,
                    interim_results=True)

                requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in stream.generator())

                responses = client.streaming_recognize(streaming_config, requests)

                while True:
                    try:
                        response = next(responses,None)
                    except Unknown_except:
                        """
                        This is a very ugly hack, the problem is that when the stream generator closes, this
                        responses generator throws an exception because the backend system cannot get anymore content.
                        I guess they never expect anyone to use it the same ways we do.
                        """
                        break
                    if not response:
                        break
                    for result in response.results:
                        for alternative in result.alternatives:
                            msg = Speech_msg()
                            msg.text = str(alternative.transcript)
                            if alternative.confidence is not None:
                                msg.confidence = float(alternative.confidence)
                            else:
                                msg.confidence = 0
                            #msg.stability = result.stability
                            msg.is_final = result.is_final
                            self._stt_pub.publish(msg)
            
                if not rospy.is_shutdown() and self._enable_flag:
                    rospy.loginfo("restarting google speech api due to time limit")
            else:
                #wait for it to be enabled
                self._enable_event.wait()
                #enable event is enabled
                stream.start()

def main():
    rospy.init_node('stt_node')
    asc = AudioStreamingController()

    def shutdown_handler():
        asc._enable_event.set()
    rospy.on_shutdown(shutdown_handler)
    
    rospy.loginfo('initialization complete')
    asc.spin()
   

if __name__ == '__main__':
    main()