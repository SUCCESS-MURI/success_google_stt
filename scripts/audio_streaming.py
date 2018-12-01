
import queue
import time

import alsaaudio
import rospy 
from audio_common_msgs.msg import (
    AudioData
)

class AudioStreamingObject():
    """This is the template for the different ways to 
    stream audio to the module
    """

    def start(self):
        """This will be called when the programmer wants the stream to start. This must be called 
        before any generator function is called
        """
        raise NotImplementedError()

    def close(self):
        """Closes the stream and clear any resources
        """
        raise NotImplementedError()
    
    def generator(self):
        """The generator called by Google's API. Must yield chunks of audio
        """
        raise NotImplementedError()
    

class AudioStreamingObjectDirect(AudioStreamingObject):
    """The audio is streamed directly from the microphone without any ROS topic in between
    """

    def __init__(self):
        self.closed = True

    def start(self):
        self.closed = False
        self._inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK)
        self._inp.setchannels(1)
        self._inp.setrate(16000)
        self._inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        self._inp.setperiodsize(512)


    def close(self):
        self.closed = True
        self._inp.close()


    def generator(self):
        while not self.closed:
            try:
                cur_l, cur_data = self._inp.read()
                while(cur_l <= 0) and not self.closed:
                    time.sleep(.001)
                    cur_l, cur_data = self._inp.read()
            except Exception as exp:
                rospy.logerror('Exception raised in AudioStreamingObjectDirect:{}'.format(exp))
            yield cur_data


class AudioStreamingObjectTopic(AudioStreamingObject):
    """The audio is streamed from the rostopic /audio instead of directly from the 
    microphone.
    """

    def __init__(self):
        #create a buffer to store the audio
        #subscribe to audio
        self.closed = True
        self._buffer = queue.Queue()
        rospy.Subscriber('audio', AudioData, self._audio_cb)


    def _audio_cb(self, msg):
        self._buffer.put(msg.data)

    def start(self):
        with self._buffer.mutex:
            self._buffer.queue.clear() #clear the buffer
        self.closed = False 

    def generator(self):
        while not self.closed:

            data =[self._buffer.get()]
            while not self._buffer.empty() and not self.closed:
                data.append(self._buffer.get(block=False))

            if self.closed:
                return 
            yield b''.join(data)

    def close(self):
        self.closed = True
