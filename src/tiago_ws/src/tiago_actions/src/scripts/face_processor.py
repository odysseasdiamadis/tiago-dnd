#!/usr/bin/env python

import rospy
import numpy as np
import tempfile
import os
from typing import Optional, Tuple, List
from PIL import Image as PILImage
from deepface import DeepFace
from detection import compare_embeddings, scale_bbox
from player_model import Player
import torch

class FaceProcessor:
    def __init__(self, similarity_threshold: float = 0.85, border_margin: int = 50, scale_factor: float = 1):
        self.similarity_threshold = similarity_threshold
        self.border_margin = 50  # pixels to avoid faces on image borders
        self.scale_factor = scale_factor
        print("[FACEPROCESSOR]: CUDA AVAILABLE: " + str(torch.cuda.is_available()))
        rospy.loginfo("Preloading Facenet model...")
        self.model = DeepFace.build_model("Facenet")
        rospy.loginfo("Facenet model preloaded successfully")

    
    def extract_face_embedding(self, image: PILImage.Image, bbox: Tuple[float, float, float, float]) -> Optional[np.ndarray]:
        x1, y1, x2, y2 = map(int, bbox)
        image_width, image_height = image.size
        
        # Check if face is too close to image border
        if (x1 <= self.border_margin or y1 <= self.border_margin or 
            x2 >= (image_width - self.border_margin) or y2 >= (image_height - self.border_margin)):
            rospy.loginfo(f"Skipping face on border: bbox=({x1}, {y1}, {x2}, {y2})")
            return None
        
        try:
            rospy.loginfo("Scaling and cropping face...")
            x1_s, y1_s, x2_s, y2_s = scale_bbox(x1, y1, x2, y2, scale_factor=self.scale_factor, 
                                                 image_width=image_width, image_height=image_height)
            cropped_face = image.crop((x1_s, y1_s, x2_s, y2_s))
            
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
                cropped_face.save(tmp.name)
                tmp_path = tmp.name
            
            try:
                rospy.loginfo("Extracting face embedding...")
                embedding_data = DeepFace.represent(tmp_path, model_name="Facenet", enforce_detection=False)
                if embedding_data:
                    embedding = np.array(embedding_data[0]['embedding'])
                    rospy.loginfo("Face embedding extracted successfully!!")
                    return embedding
                else:
                    rospy.logwarn("DeepFace returned empty embedding data")
                    return None
                    
            except Exception as e:
                rospy.logerr(f"Error in DeepFace embedding extraction: {e}")
                return None
            finally:
                os.remove(tmp_path)
                
        except Exception as e:
            rospy.logerr(f"Error in face processing: {e}")
            return None
    
    def find_matching_player(self, face_embedding: np.ndarray, known_players: List[Player]) -> Optional[int]:
        for player in known_players:
            player_embedding = np.array(player.face_embedding)
            similarity = compare_embeddings(player_embedding, face_embedding)
            
            rospy.loginfo(f"Similarity with player {player.player_id}: {similarity:.3f}")
            
            if similarity > self.similarity_threshold:
                return player.player_id
        
        return None
    
    def should_update_player_position(self, new_bbox: Tuple[float, float, float, float], current_player: Player, image_width: int) -> bool:
        # Calculate how far each face center is from image center
        new_center_x = (new_bbox[0] + new_bbox[2]) / 2
        new_distance_from_center = abs(new_center_x - image_width / 2)
        
        old_center_x = (current_player.face_position[0] + current_player.face_position[2]) / 2
        old_distance_from_center = abs(old_center_x - image_width / 2)
        
        return new_distance_from_center < old_distance_from_center
    
    def create_new_player(self, face_embedding: np.ndarray, yaw: float, bbox: Tuple[float, float, float, float], player_id: int, name: str, klass: str) -> Player:
        return Player(
            face_embedding=face_embedding.tolist(),
            yaw=yaw,
            is_present=True,
            face_position=bbox,
            player_id=player_id,
            klass=klass,
            name=name
        )
    
    def update_player_data(self, player: Player, face_embedding: np.ndarray, 
                          yaw: float, bbox: Tuple[float, float, float, float]) -> Player:
        """Update existing player with new detection data."""
        player.face_embedding = face_embedding.tolist()
        player.yaw = yaw
        player.face_position = bbox
        return player
