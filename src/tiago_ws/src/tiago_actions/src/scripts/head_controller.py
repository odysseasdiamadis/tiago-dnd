#!/usr/bin/env python

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Optional, Tuple
from supervision import Detections
from ultralytics import YOLO
from PIL import Image as PILImage
from player_model import Player, PlayerDatabase

from detection import detect_face
from detection import compare_embeddings, download_model, scale_bbox, detect_face
from face_processor import FaceProcessor
from sensor_msgs.msg import JointState

class HeadController:
    """Controls TIAGo head movement for player detection and tracking."""
    
    def __init__(self, image_topic: str = '/xtion/rgb/image_color'):
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.bridge = CvBridge()
        self.image_topic = image_topic
        
        # Control parameters
        self.center_tolerance = 20  # pixels tolerance for face centering
        self.max_centering_attempts = 10
        self.centering_step_size = 0.05  # radians for fine adjustments
        self.movement_duration = 0.3  # seconds for head movements
        
        # Image properties (should match camera specs)
        self.image_width = 640
        self.image_height = 480
        self.image_center_x = self.image_width / 2
        
        rospy.sleep(0.5)  # Wait for publisher to initialize
    
        self.face_processor = FaceProcessor(
            similarity_threshold=0.9, # threshold for cosine similarity
            scale_factor=1,           # scale factor of the cropped face
            border_margin=50,         # margin from the borders of the camera from which to consider faces
        )
        # Load YOLO model
        model_path = download_model("arnabdhar/YOLOv8-Face-Detection", "model.pt")
        self.model = YOLO(model_path)


    def move_head(self, yaw: float, pitch: float = 0.0, duration: float = None) -> None:
        """Move TIAGo head to specified yaw and pitch angles."""
        if duration is None:
            duration = self.movement_duration
            
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [yaw, pitch]
        point.time_from_start = rospy.Duration(duration)
        traj.points = [point]
        self.head_pub.publish(traj)
        rospy.sleep(duration + 0.1)  # Wait for movement to complete
    
    def capture_image(self, timeout: float = 3.0) -> Optional[np.ndarray]:
        """Captures image from the camera."""
        try:
            ros_img = rospy.wait_for_message(self.image_topic, Image, timeout=timeout)
            cv_img = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding='bgr8')
            return cv_img
        except rospy.ROSException as e:
            rospy.logwarn(f"Timeout waiting for image: {e}")
            return None
    
    def detect_faces_in_current_view(self, model) -> Tuple[Optional[np.ndarray], Detections]:
        """Detect faces in the current camera view."""
        image = self.capture_image()
        if image is None:
            return None, Detections.empty()
        
        # Convert to PIL for YOLO detection
        from PIL import Image as PILImage
        pil_image = PILImage.fromarray(image)
        detections = detect_face(model, pil_image)
        
        return image, detections
    
    def get_face_center_error(self, bbox: Tuple[float, float, float, float]) -> float:
        """calculate horizontal distance from face center to image center."""
        x1, y1, x2, y2 = bbox
        face_center_x = (x1 + x2) / 2
        return face_center_x - self.image_center_x
    
    def is_face_centered(self, bbox: Tuple[float, float, float, float]) -> bool:
        """Check if face is centered within tolerance."""
        error = abs(self.get_face_center_error(bbox))
        return error <= self.center_tolerance
    
    def center_face_in_view(self, model, current_yaw: float, pitch: float = 0.0) -> Tuple[Optional[float], Optional[Tuple]]:
        rospy.loginfo(f"Starting face centering at yaw {current_yaw:.2f}")
        
        for attempt in range(self.max_centering_attempts):
            image, detections = self.detect_faces_in_current_view(model)
            
            if len(detections.xyxy) == 0:
                rospy.loginfo(f"No faces detected in attempt {attempt + 1}")
                return None, None
            
            # Take the largest face (most prominent)
            largest_face_idx = 0
            if len(detections.xyxy) > 1:
                areas = [(x2-x1)*(y2-y1) for x1, y1, x2, y2 in detections.xyxy]
                largest_face_idx = np.argmax(areas)
            
            bbox = detections.xyxy[largest_face_idx]
            x1, y1, x2, y2 = bbox
            
            # Check if face is already centered
            if self.is_face_centered(bbox):
                rospy.loginfo(f"Face centered successfully at yaw {current_yaw:.2f}")
                return current_yaw, tuple(bbox)
            
            # calculate correction needed
            error = self.get_face_center_error(bbox)
            
            # Simple proportional control
            yaw_correction = -error * (self.centering_step_size / 100)  # Convert pixel error to radians
            new_yaw = current_yaw + yaw_correction
            
            # Clamp yaw to reasonable limits
            new_yaw = max(-1.5, min(1.5, new_yaw))
            
            rospy.loginfo(f"Attempt {attempt + 1}: Face center error = {error:.1f}px, adjusting yaw by {yaw_correction:.3f} rad")
            
            # Move head to a new position
            self.move_head(new_yaw, pitch, duration=0.2)
            current_yaw = new_yaw
            
            rospy.sleep(0.1)  # Small delay for stabilization
        
        rospy.logwarn(f"Failed to center face after {self.max_centering_attempts} attempts")
        return None, None
    
    def calculate_accurate_yaw_for_face(self, bbox: Tuple[float, float, float, float], current_yaw: float) -> float:
        # Calculate pixel error from image center
        error_pixels = self.get_face_center_error(bbox)
        
        # Convert pixel error to angular error
        # NOTE: Assuming camera field of view is approximately 60 degrees (1.047 radians)
        camera_fov_radians = 1.047
        angular_error = (error_pixels / self.image_width) * camera_fov_radians
        
        # Calculate target yaw
        target_yaw = current_yaw - angular_error
        
        # Clamp to reasonable limits
        target_yaw = max(-1.5, min(1.5, target_yaw))
        
        return target_yaw

    def scan_for_faces_with_accurate_positioning(self, model, start_yaw: float, end_yaw: float, scan_step: float = 0.2) -> list:

        face_data = []
        current_yaw = start_yaw
        
        rospy.loginfo(f"Scanning for faces with accurate positioning from {start_yaw:.2f} to {end_yaw:.2f} rad")
        
        while current_yaw <= end_yaw:
            self.move_head(current_yaw, 0.0)
            
            image, detections = self.detect_faces_in_current_view(model)
            
            if len(detections.xyxy) > 0:
                # Process all detected faces in this view
                for i, bbox in enumerate(detections.xyxy):
                    # Calculate precise yaw to center this face
                    target_yaw = self.calculate_accurate_yaw_for_face(bbox, current_yaw)
                    
                    rospy.loginfo(f"Face detected at scan yaw {current_yaw:.2f}, calculated target yaw {target_yaw:.2f}")
                    face_data.append((target_yaw, tuple(bbox)))
            
            current_yaw += scan_step
        
        rospy.loginfo(f"Scan complete. Found {len(face_data)} faces with calculated positions")
        return face_data

    def get_head_joints(self, timeout=1.0):
        try:
            # Questa funzione BLOCCA l'esecuzione finché non arriva un messaggio
            msg = rospy.wait_for_message('/joint_states', JointState, timeout=timeout)
            if not msg:
                raise ValueError("Could not find joint states")
            # I joint in 'msg.name' non hanno sempre lo stesso ordine, 
            # quindi dobbiamo cercare l'indice corretto ogni volta.
            if "head_1_joint" in msg.name and "head_2_joint" in msg.name:
                idx_1 = msg.name.index("head_1_joint") # Yaw (Pan)
                idx_2 = msg.name.index("head_2_joint") # Pitch (Tilt)
                
                current_yaw = msg.position[idx_1]
                current_pitch = msg.position[idx_2]
                
                return current_yaw, current_pitch
            else:
                rospy.logwarn("I joint della testa non sono presenti nel messaggio /joint_states")
                return None, None

        except rospy.ROSException:
            rospy.logerr("Timeout: Nessun messaggio ricevuto da /joint_states entro {:.1f}s".format(timeout))
            return None, None
        except ValueError:
            rospy.logerr("Errore nel parsing dei joint.")
            return None, None

    def look_at_player(self, player: Player, assert_player_is_here=False) -> bool:
        if player is None:
            rospy.logwarn(f"Player {player.player_id} not found")
            return False
        player_db = PlayerDatabase('players_database.json')
        players = player_db.load_players()
        rospy.loginfo(f"Looking at player {player.player_id}")
        bbox_tolerance = 50
        self.move_head(player.yaw, 0.0)
        if not assert_player_is_here:
            return True
        image, detections = self.detect_faces_in_current_view(self.model)
        pil_image = PILImage.fromarray(image)
        # Find the player among all detections
        for detection in detections.xyxy:
            emb = self.face_processor.extract_face_embedding(pil_image, detection)
            if emb is None:
                rospy.loginfo("Player not found!")
                return False
            p_id = self.face_processor.find_matching_player(emb, players)
            if p_id == player.player_id:
                # Found it, so We check for bounding box
                x1, y1, x2, y2 = detection
                if x1 < bbox_tolerance or x2 > 640 - bbox_tolerance:
                    return True
                current_yaw, _ = self.get_head_joints()
                assert current_yaw is not None
                right_yaw = self.calculate_accurate_yaw_for_face(detection, current_yaw)
                player.yaw = right_yaw

                self.move_head(right_yaw)
                return True
            else:
                return False

        return False


    def scan_for_faces(self, model, start_yaw: float, end_yaw: float, scan_step: float = 0.2) -> list:

        face_yaws = []
        current_yaw = start_yaw
        
        rospy.loginfo(f"Scanning for faces from {start_yaw:.2f} to {end_yaw:.2f} rad")
        
        while current_yaw <= end_yaw:
            self.move_head(current_yaw, 0.0)
            
            image, detections = self.detect_faces_in_current_view(model)
            
            if len(detections.xyxy) > 0:
                rospy.loginfo(f"Face detected at yaw {current_yaw:.2f}")
                face_yaws.append(current_yaw)
            
            current_yaw += scan_step
        
        rospy.loginfo(f"Scan complete. Found faces at {len(face_yaws)} positions")
        return face_yaws