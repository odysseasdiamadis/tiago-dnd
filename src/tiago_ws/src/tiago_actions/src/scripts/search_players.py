#!/usr/bin/env python

from dataclasses import dataclass
from typing import Any, Union
import rospy
import numpy as np
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
from huggingface_hub import hf_hub_download
from ultralytics import YOLO
from supervision import Detections
from PIL import Image as PILImage
import numpy as np
from deepface import DeepFace
from scipy.spatial.distance import cosine

from detection import compare_embeddings, download_model, scale_bbox, detect_face

@dataclass
class Player:
    face: Any
    yaw: float
    face_position: tuple

class HeadSweepImageCapture:
    def __init__(self, start=-1.0, end=1.0, steps=10, pitch=0.0):
        rospy.init_node('head_sweep_image_capture', anonymous=True)

        # self.start = start
        # self.end = end
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.bridge = CvBridge()
        self.image_topic = '/xtion/rgb/image_color'
        self.pitch = pitch

        # Compute yaw angles
        self.yaws = np.linspace(start, end, steps)
        self.players = []
        model_path: str = download_model("arnabdhar/YOLOv8-Face-Detection", "model.pt")

        self.model = YOLO(model_path)
        rospy.sleep(1.0)  # Wait for connections

    def move_head(self, yaw, pitch):
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [yaw, pitch]
        point.time_from_start = rospy.Duration(0.5)
        traj.points = [point]
        self.head_pub.publish(traj)

    def extract_faces_and_embeddings(self, np_image: np.ndarray, model) -> list:
        """
        Detects faces and returns list of (bounding_box, embedding) pairs.
        
        Args:
            np_image (np.ndarray): Input image as a NumPy array (BGR or RGB).
            model: YOLOv8 face detection model.

        Returns:
            List of tuples: [((x1, y1, x2, y2), embedding), ...]
        """
        import tempfile
        import os

        # Convert np_image to PIL Image
        image = PILImage.fromarray(np_image)
        image_width, image_height = image.size

        # Detect faces
        rospy.loginfo("Detecting faces...")
        detections = detect_face(model, image)
        rospy.loginfo("ok")

        faces_and_embeddings = []

        for bbox in detections.xyxy:
            x1, y1, x2, y2 = map(int, bbox)

            # Scale and crop
            rospy.loginfo("Scaling...")
            x1_s, y1_s, x2_s, y2_s = scale_bbox(x1, y1, x2, y2, scale_factor=1.3, image_width=image_width, image_height=image_height)
            cropped_face = image.crop((x1_s, y1_s, x2_s, y2_s))
            rospy.loginfo("Ok")

            # Save to temp file for DeepFace
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
                cropped_face.save(tmp.name)
                tmp_path = tmp.name

            try:
                rospy.loginfo("Extracting features...")
                embedding_data = DeepFace.represent(tmp_path, model_name="Facenet", enforce_detection=False)
                if embedding_data:
                    embedding = np.array(embedding_data[0]['embedding'])
                    faces_and_embeddings.append(((x1, y1, x2, y2), embedding))
                rospy.loginfo("Ok")
                
            except Exception as e:
                print(f"Error in DeepFace embedding extraction: {e}")
            finally:
                os.remove(tmp_path)

        return faces_and_embeddings



    def capture_image(self):
        try:
            ros_img = rospy.wait_for_message(self.image_topic, Image, timeout=3.0)
            cv_img = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding='bgr8')
            return cv_img
        except rospy.ROSException as e:
            rospy.logwarn(f"Timeout waiting for image: {e}")
            return None

    def look_player(self, i: int):
        player = self.players[i]
        self.move_head(player.yaw, self.pitch)

    def run(self):
        self.move_head(self.yaws[0], self.pitch)
        rospy.sleep(3)
        WIDTH = 640
        FINAL_PLAYER_LOOK_IDX = 0 # id of the player to look at after the detection
        SIMILARITY_THRESHOLD = 0.85

        for i, yaw in enumerate(self.yaws):
            rospy.loginfo(f"YAW={yaw:.2f} STEP ({i+1}/{len(self.yaws)})")
            self.move_head(yaw, self.pitch)
            # rospy.sleep(0.5)  # Allow time for head to reach position

            image = self.capture_image()
            if image is not None:
                # faces = self.get_faces(image=image)
                faces = self.extract_faces_and_embeddings(image, model=self.model)
                new_players = []
                for bounding_box, embedding in faces:
                    print(f"LEN FACES: {len(faces)}\tBB: {bounding_box}\tEMBEDDING SHAPE: {embedding.shape}")
                    if len(self.players) == 0:
                        rospy.loginfo(f"Goodmorning player 1!")
                        new_players.append(Player(embedding, yaw, face_position=bounding_box))
                    else:
                        player_idx = -1
                        # Search for an existing player
                        for idx, player in enumerate(self.players):
                            similarity = compare_embeddings(player.face, embedding2=embedding)
                            rospy.loginfo(f"SIMILARITY BETWEEN CURRENT FACE AND PLAYER {idx}: {similarity}")
                            if similarity > SIMILARITY_THRESHOLD:
                                player_idx = idx

                        if player_idx < 0:
                            rospy.loginfo(f"Welcome player {len(self.players)}!")
                            new_players.append(Player(embedding, yaw=yaw, face_position=bounding_box))
                        else:
                            player = self.players[player_idx]
                            rospy.loginfo("We already know them")
                            new_p = abs(bounding_box[0] - WIDTH/2)
                            old_p = abs(player.face_position[0] - WIDTH/2)
                            rospy.loginfo(f"new_p: {new_p}, old_p: {old_p}")
                            
                            if new_p < old_p:
                                player.face = embedding
                                player.face_position = bounding_box
                                player.yaw = yaw
                                rospy.loginfo("Substituting a player!")

                self.players += new_players

                rospy.loginfo(f"got {len(faces)} faces")
            else:
                rospy.logwarn("No image captured at this position.")
        rospy.loginfo(f"Found a total of {len(self.players)} players")
        rospy.sleep(0.5)
        rospy.loginfo(f"Looking at player {FINAL_PLAYER_LOOK_IDX+1}")
        self.look_player(FINAL_PLAYER_LOOK_IDX)

if __name__ == '__main__':
    try:
        node = HeadSweepImageCapture(start=-1.0, end=1.0, steps=20) # n of steps must be tuned in order to _not_ capture a face in half
        node.run()
    except rospy.ROSInterruptException:
        pass
