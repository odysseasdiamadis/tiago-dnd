#!/usr/bin/env python

import rospy
import numpy as np
from PIL import Image as PILImage
from ultralytics import YOLO
from typing import List, Optional

from detection import download_model
from player_model import Player, PlayerDatabase
from head_controller import HeadController
from face_processor import FaceProcessor


class EnhancedPlayerSearcher:
    """
    Enhanced player search system that implements:
    1. Horizontal head scanning with face centering control
    2. Embedding computation only when faces are properly centered
    3. JSON persistence for player database
    4. Sequential left-to-right player discovery
    """
    
    def __init__(self, scan_start: float = -1.0, scan_end: float = 1.0, 
                 scan_step: float = 0.3, players_file: str = 'players_database.json'):
        rospy.init_node('enhanced_player_searcher', anonymous=True)
        
        # Initialize components
        self.head_controller = HeadController()
        self.face_processor = FaceProcessor()
        self.player_db = PlayerDatabase(players_file)
        
        # Load existing players
        self.players = self.player_db.load_players()
        rospy.loginfo(f"Loaded {len(self.players)} players from database")
        
        # Scanning parameters
        self.scan_start = scan_start
        self.scan_end = scan_end
        self.scan_step = scan_step
        
        # Load YOLO model
        model_path = download_model("arnabdhar/YOLOv8-Face-Detection", "model.pt")
        self.yolo = YOLO(model_path)
        
        rospy.loginfo("Enhanced Player Searcher initialized")
    
    def search_and_analyze_players(self) -> List[Player]:
        """
        Main method to search for players using the enhanced algorithm:
        1. Scan horizontally for faces
        2. For each face, center it in view using control law
        3. Extract embedding only when properly centered
        4. Update player database
        
        Returns:
            List of all known players (existing + newly discovered)
        """
        rospy.loginfo("Starting enhanced player search...")
        
        # Start from leftmost position
        self.head_controller.move_head(self.scan_start, 0.0, duration=1.0)
        
        # Scan for face positions
        # face_yaws = self.head_controller.scan_for_faces(
        #     self.yolo, self.scan_start, self.scan_end, self.scan_step
        # )
        
        face_yaws = []
        current_yaw = self.scan_start

        while current_yaw <= self.scan_end:
            self.head_controller.move_head(current_yaw, 0.0)
            
            image, detections = self.head_controller.detect_faces_in_current_view(self.yolo)
            
            if len(detections.xyxy) > 0:
                
                rospy.loginfo(f"Face detected at yaw {current_yaw:.2f}")
                face_yaws.append(current_yaw)
            
            current_yaw += self.scan_step

        
        if not face_yaws:
            rospy.loginfo("No faces detected during scan")
            return self.players
        
        rospy.loginfo(f"Found faces at {len(face_yaws)} positions. Proceeding to detailed analysis...")
        
        # Process each detected face position
        new_players_count = 0
        for i, yaw in enumerate(face_yaws):
            rospy.loginfo(f"Processing face {i+1}/{len(face_yaws)} at yaw {yaw:.2f}")
            
            # Center the face in view
            final_yaw, centered_bbox = self.head_controller.center_face_in_view(
                self.yolo, yaw, pitch=0.0
            )
            
            if final_yaw is None or centered_bbox is None:
                rospy.logwarn(f"Failed to center face at yaw {yaw:.2f}, skipping...")
                continue
            
            # Capture image with centered face
            image = self.head_controller.capture_image()
            if image is None:
                rospy.logwarn("Failed to capture image after centering")
                continue
            
            # Convert to PIL for processing
            pil_image = PILImage.fromarray(image)
            
            # Extract face embedding
            face_embedding = self.face_processor.extract_face_embedding(pil_image, centered_bbox)
            if face_embedding is None:
                rospy.logwarn("Failed to extract face embedding")
                continue
            
            # Check if this is a known player
            matching_player_id = self.face_processor.find_matching_player(face_embedding, self.players)
            
            if matching_player_id is not None:
                # Known player - just greet them, don't save new embedding
                rospy.loginfo(f"Welcome back player {matching_player_id}!")
                print(f"\n🎲 Welcome back player {matching_player_id}! 🎲\n")
                
                # Optionally update position if this view is significantly better centered
                matching_player = next(p for p in self.players if p.player_id == matching_player_id)
                
                if self.face_processor.should_update_player_position(
                    centered_bbox, matching_player, self.head_controller.image_width
                ):
                    # Only update position data, not the face embedding
                    matching_player.yaw = final_yaw
                    matching_player.face_position = centered_bbox
                    self.player_db.update_player(matching_player_id, matching_player)
                    rospy.loginfo(f"Updated position for player {matching_player_id}")
            else:
                # New player - save face embedding and create entry
                new_player_id = self.player_db.get_next_player_id()
                new_player = self.face_processor.create_new_player(
                    face_embedding, final_yaw, centered_bbox, new_player_id
                )
                
                self.players.append(new_player)
                self.player_db.add_player(new_player)
                new_players_count += 1
                
                rospy.loginfo(f"Hello player {new_player_id}! Nice to meet you!")
                print(f"\n🎲 Hello player {new_player_id}! Nice to meet you! 🎲\n")
            
            # Small delay between players
            rospy.sleep(0.5)
        
        rospy.loginfo(f"Player search complete. Found {new_players_count} new players. "
                     f"Total players: {len(self.players)}")
        
        return self.players
    
    def look_at_player(self, player_id: int) -> bool:
        """Look at a specific player by their ID."""
        player = next((p for p in self.players if p.player_id == player_id), None)
        if player is None:
            rospy.logwarn(f"Player {player_id} not found")
            return False
        
        rospy.loginfo(f"Looking at player {player_id}")
        self.head_controller.move_head(player.yaw, 0.0)
        return True
    
    def get_player_summary(self) -> str:
        """Get a summary of all known players."""
        if not self.players:
            return "No players found."
        
        summary = f"Found {len(self.players)} players:\n"
        for player in sorted(self.players, key=lambda p: p.player_id):
            summary += f"  Player {player.player_id}: yaw={player.yaw:.2f}, discovered={player.discovered_time}\n"
        
        return summary
    
    def run_search_demo(self):
        """Run the complete player search and demonstration."""
        rospy.loginfo("Starting player search demonstration...")
        
        # Search for players
        players = self.search_and_analyze_players()
        
        # Print summary
        rospy.loginfo(self.get_player_summary())
        
        if players:
            # Look at each player in sequence
            rospy.loginfo("Looking at each player...")
            for player in sorted(players, key=lambda p: p.player_id):
                rospy.loginfo(f"Looking at player {player.player_id}")
                self.look_at_player(player.player_id)
                rospy.sleep(2.0)
            
            # Return to first player
            rospy.loginfo("Returning to first player")
            self.look_at_player(0)
        
        rospy.loginfo("Player search demonstration complete!")


if __name__ == '__main__':
    try:
        searcher = EnhancedPlayerSearcher(
            scan_start=-1.2, 
            scan_end=1.2, 
            scan_step=0.3,
            players_file='players_database.json'
        )
        searcher.run_search_demo()
    except rospy.ROSInterruptException:
        pass