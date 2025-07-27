#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pal_detection_msgs.msg import Detections2d
import yaml

def callback(data):

    rospy.loginfo(rospy.get_caller_id() + " I heard: ")
    rospy.loginfo(f"I got {len(data.detections)}, which are:")
    for detection in data.detections:
        rospy.loginfo(f"X={detection.x}, Y={detection.y}, WxH = {detection.width}x{detection.height}")


def main():
    rospy.init_node('hello_node')
    rospy.loginfo("Hello from my TIAGo Python node!")
    rospy.Subscriber("/person_detector/detections", Detections2d, callback)
    rospy.spin()

if __name__ == '__main__':
    main()

