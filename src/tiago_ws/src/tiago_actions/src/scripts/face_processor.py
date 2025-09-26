#!/usr/bin/env python

import rospy
import numpy as np
import tempfile
import os
from typing import Optional, Tuple, List
from PIL import Image as PILImage
from deepface import DeepFace

from detection import scale_bbox, compare_embeddings
from player_model import Player


class FaceProcessor:
    """Handles face embedding extraction and player recognition."""
    
    def __init__(self, similarity_threshold: float = 0.85, border_margin: int = 50, scale_factor: float = 1):
        self.similarity_threshold = similarity_threshold
        self.border_margin = 50  # pixels to avoid faces on image borders
        self.scale_factor = scale_factor
    
    def extract_face_embedding(self, image: PILImage.Image, bbox: Tuple[float, float, float, float]) -> Optional[np.ndarray]:
        """
        Extract face embedding from a detected face in the image.
        
        Args:
            image: PIL Image containing the face
            bbox: Bounding box coordinates (x1, y1, x2, y2)
            
        Returns:
            Face embedding as numpy array, or None if extraction failed
        """
        x1, y1, x2, y2 = map(int, bbox)
        image_width, image_height = image.size
        
        # Check if face is too close to image border
        if (x1 <= self.border_margin or y1 <= self.border_margin or 
            x2 >= (image_width - self.border_margin) or y2 >= (image_height - self.border_margin)):
            rospy.loginfo(f"Skipping face on border: bbox=({x1}, {y1}, {x2}, {y2})")
            return None
        
        # Scale and crop face
        try:
            rospy.loginfo("Scaling and cropping face...")
            x1_s, y1_s, x2_s, y2_s = scale_bbox(x1, y1, x2, y2, scale_factor=self.scale_factor, 
                                                 image_width=image_width, image_height=image_height)
            cropped_face = image.crop((x1_s, y1_s, x2_s, y2_s))
            
            # Save to temporary file for DeepFace
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
                cropped_face.save(tmp.name)
                tmp_path = tmp.name
            
            try:
                rospy.loginfo("Extracting face embedding...")
                embedding_data = DeepFace.represent(tmp_path, model_name="Facenet", enforce_detection=False)
                if embedding_data:
                    embedding = np.array(embedding_data[0]['embedding'])
                    rospy.loginfo("Face embedding extracted successfully")
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
        """
        Find if the face embedding matches any known player.
        
        Args:
            face_embedding: Face embedding to match
            known_players: List of known players
            
        Returns:
            Player ID if match found, None otherwise
        """
        for player in known_players:
            player_embedding = np.array(player.face_embedding)
            similarity = compare_embeddings(player_embedding, face_embedding)
            
            rospy.loginfo(f"Similarity with player {player.player_id}: {similarity:.3f}")
            
            if similarity > self.similarity_threshold:
                return player.player_id
        
        return None
    
    def should_update_player_position(self, new_bbox: Tuple[float, float, float, float], 
                                     current_player: Player, image_width: int) -> bool:
        """
        Determine if the new face position is better centered than the current one.
        
        Args:
            new_bbox: New face bounding box
            current_player: Current player data
            image_width: Image width in pixels
            
        Returns:
            True if new position is better (more centered)
        """
        # Calculate how far each face center is from image center
        new_center_x = (new_bbox[0] + new_bbox[2]) / 2
        new_distance_from_center = abs(new_center_x - image_width / 2)
        
        old_center_x = (current_player.face_position[0] + current_player.face_position[2]) / 2
        old_distance_from_center = abs(old_center_x - image_width / 2)
        
        return new_distance_from_center < old_distance_from_center
    
    def create_new_player(self, face_embedding: np.ndarray, yaw: float, 
                         bbox: Tuple[float, float, float, float], player_id: int) -> Player:
        """Create a new Player object from detection data."""
        return Player(
            face_embedding=face_embedding.tolist(),
            yaw=yaw,
            is_present=True,
            face_position=bbox,
            player_id=player_id
        )
    
    def update_player_data(self, player: Player, face_embedding: np.ndarray, 
                          yaw: float, bbox: Tuple[float, float, float, float]) -> Player:
        """Update existing player with new detection data."""
        player.face_embedding = face_embedding.tolist()
        player.yaw = yaw
        player.face_position = bbox
        return player