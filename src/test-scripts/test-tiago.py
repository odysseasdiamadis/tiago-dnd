#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

def send_head_trajectory():
    # Initialize the ROS node
    rospy.init_node('head_trajectory_publisher', anonymous=True)

    # Create the publisher
    pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)

    # Wait for publisher connection (optional but good practice)
    rospy.sleep(1)

    # Create the JointTrajectory message
    traj_msg = JointTrajectory()

    # Fill in the header
    traj_msg.header = Header()
    traj_msg.header.stamp = rospy.Time(0)  # Equivalent to secs: 0, nsecs: 0

    # Specify the joint names
    traj_msg.joint_names = ['head_1_joint', 'head_2_joint']

    # Create the trajectory point
    point = JointTrajectoryPoint()
    point.positions = [0.5, -0.5]
    point.velocities = [0.0, 0.0]
    point.time_from_start = rospy.Duration(2.0)  # 2 seconds

    # Add the point to the trajectory
    traj_msg.points.append(point)

    # Publish the trajectory once
    pub.publish(traj_msg)
    rospy.loginfo("Published head trajectory command.")

if __name__ == '__main__':
    try:
        send_head_trajectory()
    except rospy.ROSInterruptException:
        pass
