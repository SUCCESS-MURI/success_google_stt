#!/usr/bin/python

import rospy
from std_srvs.srv import SetBool
from success_ros_msgs.msg import(
    Speech
)

def stt_cb(msg):
    rospy.loginfo(msg)


def main():

    #enable STT
    rospy.init_node('stt_test_node')
    rospy.wait_for_service('success_google_stt/toggle_stt')
   
    rospy.Subscriber('success_google_stt/stt', Speech, stt_cb, queue_size=1)

    stt_toggle = rospy.ServiceProxy('success_google_stt/toggle_stt', SetBool)
    resp = stt_toggle(True)
    rospy.loginfo('enabled STT: {}'.format(resp))
    rospy.sleep(5)
    rospy.loginfo("Disabling STT")
    stt_toggle(False)
    rospy.sleep(2)
    rospy.loginfo("Re-enabling STT")
    stt_toggle(True)
    rospy.sleep(10)
    stt_toggle(False)
    rospy.loginfo("Disabling STT")

if __name__ == '__main__':
    main()