#!/usr/bin/env python

import rospy

def main():
    rospy.init_node('hello_node')
    rospy.loginfo("Hello from my TIAGo Python node!")
    rospy.spin()

if __name__ == '__main__':
    main()

