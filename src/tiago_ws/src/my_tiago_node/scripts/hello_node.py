#!/usr/bin/env python

from dataclasses import dataclass
from typing import Any, Union
import rospy
import numpy as np
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import face_recognition

@dataclass
class Player:
    face: Any
    yaw: float
    face_position: tuple

class HeadSweepImageCapture:
    def __init__(self, start=-1.0, end=1.0, steps=10, pitch=0.0):
        rospy.init_node('head_sweep_image_capture', anonymous=True)

        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.bridge = CvBridge()
        self.image_topic = '/xtion/rgb/image_color'
        self.pitch = pitch

        # Compute yaw angles
        self.yaws = np.linspace(start, end, steps)
        self.players = []

        rospy.sleep(1.0)  # Wait for connections

    def move_head(self, yaw, pitch):
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [yaw, pitch]
        point.time_from_start = rospy.Duration(1.0)
        traj.points = [point]
        self.head_pub.publish(traj)

    def capture_image(self):
        try:
            ros_img = rospy.wait_for_message(self.image_topic, Image, timeout=3.0)
            cv_img = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding='bgr8')
            return cv_img
        except rospy.ROSException as e:
            rospy.logwarn(f"Timeout waiting for image: {e}")
            return None
        
    def get_faces(self, image):
        locs = face_recognition.face_locations(image, number_of_times_to_upsample=2, model='hog')
        rospy.loginfo(f"Found {len(locs)} faces! Encoding...")
        encs = face_recognition.face_encodings(image, known_face_locations=locs, model='large', num_jitters=2)
        rospy.loginfo(f"Encoded {len(encs)} faces!")
        return list(zip(locs, encs))

    def look_player(self, i: int):
        player = self.players[i]
        self.move_head(player.yaw, self.pitch)

    def run(self):
        self.move_head(self.yaws[0], self.pitch)
        rospy.sleep(3)
        WIDTH = 640
        idx = 0

        for i, yaw in enumerate(self.yaws):
            rospy.loginfo(f"Moving head to yaw={yaw:.2f} ({i+1}/{len(self.yaws)})")
            self.move_head(yaw, self.pitch)
            rospy.sleep(0.5)  # Allow time for head to reach position

            image = self.capture_image()
            if image is not None:
                faces = self.get_faces(image)
                for loc, face in faces:
                    if len(self.players) == 0:
                        self.players.append(Player(face, yaw, face_position=loc))
                    else:
                        for player in self.players:
                            distance = np.linalg.norm(player.face - face)
                            if distance > 0.60: 
                                # differtent player
                                self.players.append(Player(face, yaw=yaw, face_position=loc))
                            else:
                                # same player, check for "centerness" of bounding box
                                new_p = abs(loc[1] - WIDTH//2)
                                old_p = abs(player.face_position[1] - WIDTH//2)
                                rospy.loginfo(f"new_p: {new_p}, old_p: {old_p}")
                                
                                if new_p < old_p:
                                    player.face = face
                                    player.face_position = loc
                                    player.yaw = yaw
                                    rospy.loginfo("Substituting a player!")

                rospy.loginfo(f"got {len(faces)} faces")
            else:
                rospy.logwarn("No image captured at this position.")
        rospy.loginfo(f"Found a total of {len(self.players)} players")
        rospy.sleep(0.5)
        rospy.loginfo(f"Looking at player {idx+1}")
        self.look_player(idx)

if __name__ == '__main__':
    try:
        node = HeadSweepImageCapture(start=-1.0, end=1.0, steps=20)
        node.run()
    except rospy.ROSInterruptException:
        pass
