#!/usr/bin/env python

from dataclasses import dataclass
from typing import Any, Union, List
import rospy
import numpy as np
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
from huggingface_hub import hf_hub_download
from ultralytics import YOLO
from PIL import Image as PILImage
import numpy as np
import json
from deepface import DeepFace
from scipy.spatial.distance import cosine

from detection import compare_embeddings, download_model, scale_bbox, detect_face
from detection import download_model
from player_model import Player, PlayerDatabase
from head_controller import HeadController
from hand_controller import HandController
from arm_controller import ArmController
from face_processor import FaceProcessor
from brain_server_interaction import BrainInteractor, CONFIG_YAML

class EnhancedPlayerSearcher:
    """
    Enhanced player search system that implements:
    1. Horizontal head scanning with face centering control
    2. Embedding computation only when faces are properly centered
    3. JSON persistence for player database
    4. Sequential left-to-right player discovery
    """
    
    def __init__(self, scan_start: float = -1.0, scan_end: float = 1.0, 
                 scan_step: float = 0.3, players_file: str = 'players_database.json', audio_device_idx=0):
        rospy.init_node('enhanced_player_searcher', anonymous=True)
        
        self.audio_device_idx = audio_device_idx
        # Initialize components
        self.head_controller = HeadController()
        self.face_processor = FaceProcessor(
            similarity_threshold=0.9, # threshold for cosine similarity
            scale_factor=1,           # scale factor of the cropped face
            border_margin=50,         # margin from the borders of the camera from which to consider faces
        )
        self.player_db = PlayerDatabase(players_file)
        self.arm_controller = ArmController()
        self.hand_controller = HandController()
        self.brain_interactor = BrainInteractor(CONFIG_YAML)
        
        self.bbox_tolerance = 50
        # Load existing players
        self.players = self.player_db.load_players()
        rospy.loginfo(f"Loaded {len(self.players)} players from database")
        
        # Scanning parameters
        self.scan_start = scan_start
        self.scan_end = scan_end
        self.scan_step = scan_step
        
        # Load YOLO model
        model_path = download_model("arnabdhar/YOLOv8-Face-Detection", "model.pt")
        self.model = YOLO(model_path)
        
        rospy.loginfo("Enhanced Player Searcher initialized")
    
    def search_and_analyze_players(self) -> List[Player]:
        """
        Main method to search for players using the enhanced algorithm.
        Implements complete scanning logic with face detection, embedding comparison,
        and centering of unknown players.
        
        Returns:
            List of all known players (existing + newly discovered)
        """
        rospy.loginfo("Starting enhanced player search...")
        
        # Start from leftmost position
        self.head_controller.move_head(self.scan_start, 0.0, duration=1.0)
        rospy.sleep(1.0)
        
        current_yaw = self.scan_start
        new_players_count = 0
        
        while current_yaw <= self.scan_end:
            # Move to current scan position
            self.head_controller.move_head(current_yaw, 0.0)
            
            # Detect faces in current view
            image, detections = self.head_controller.detect_faces_in_current_view(self.model)
            
            if len(detections.xyxy) > 0:
                rospy.loginfo(f"Found {len(detections.xyxy)} faces at yaw {current_yaw:.2f}")
                
                # 1) Order detections by rightmost bounding box border (x2 coordinate)
                face_data = []
                for bbox in detections.xyxy:
                    x1, y1, x2, y2 = bbox
                    if x1 < self.bbox_tolerance or x2 > 640 - self.bbox_tolerance:
                        continue
                    face_data.append((x2, bbox))  # (rightmost_x, bbox)
                
                # Sort by rightmost x coordinate (left to right order)
                face_data.sort(key=lambda x: x[0])
                
                # 2) Compute face embeddings for all detected faces
                pil_image = PILImage.fromarray(image)
                face_embeddings = []
                
                for rightmost_x, bbox in face_data:
                    embedding = self.face_processor.extract_face_embedding(pil_image, bbox)
                    if embedding is not None:
                        face_embeddings.append((rightmost_x, bbox, embedding))
                    else:
                        rospy.logwarn(f"Failed to extract embedding for face at {bbox}")
                
                # 3) Compare embeddings with known players and identify unknown faces
                unknown_faces = []
                for rightmost_x, bbox, embedding in face_embeddings:
                    matching_player_id = self.face_processor.find_matching_player(embedding, self.players)
                    
                    if matching_player_id is not None:
                        rospy.loginfo(f"Recognized known player {matching_player_id}")
                        self.players[matching_player_id].is_present = True
                    else:
                        unknown_faces.append((rightmost_x, bbox, embedding))
                        rospy.loginfo(f"Found unknown face at bbox {bbox}")
                
                # 4-7) Process unknown players one by one
                for rightmost_x, bbox, embedding in unknown_faces:
                    rospy.loginfo(f"Processing unknown player with bbox {bbox}")
                    
                    # 5) Look at the unknown player by centering its bounding box
                    target_yaw = self.head_controller.calculate_accurate_yaw_for_face(bbox, current_yaw)
                    
                    # Center the face using control law
                    final_yaw, centered_bbox = self.head_controller.center_face_in_view(
                        self.model, target_yaw, pitch=0.0
                    )
                    
                    if final_yaw is None or centered_bbox is None:
                        rospy.logwarn(f"Failed to center unknown face, skipping...")
                        continue
                    
                    # 6) Save the player with the current centered yaw
                    # self.players = self.player_db.load_players()
                    new_players_count += 1
                    rospy.loginfo(f"\n🎲 New player found! 🎲\n")
                    
                    self.brain_interactor.say(f"Ciao avventuriero! Come ti chiami? E cosa vuoi giocare oggi?", language='it')

                    name_and_class = self.brain_interactor.ask_player_name_and_class()
                    print(name_and_class)
                    # name_and_class = self.brain_interactor.ask_player_name_and_class(device_idx=self.audio_device_idx)
                    name_and_class = json.loads(name_and_class)
                    rospy.loginfo(f"Name and class: {name_and_class}")

                    new_player_id = self.player_db.get_next_player_id()
                    new_player = self.face_processor.create_new_player(
                        embedding, final_yaw, centered_bbox, new_player_id, name_and_class['name'], name_and_class['class']
                    )
                    
                    self.player_db.add_player(new_player)
                    rospy.loginfo(f"New player: {new_player}")
                    self.brain_interactor.welcome_new_player(new_player)

                    # self.arm_controller.point_at_player(new_player, arm_distance=0.8, keep_elbow_down=True)
                    # Small delay between players
                    # rospy.sleep(0.5)
                
                # 8) All faces are now known, continue scanning
                rospy.loginfo(f"All faces at yaw {current_yaw:.2f} are now known players")
            
            # Move to next scan position
            current_yaw += self.scan_step
        
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
        self.player_db.save_players(players)
        
        # # # Print summary
        rospy.loginfo(self.get_player_summary())
        players_to_look = [p for p in self.players if p.is_present == True]

        if players_to_look:
            player = players_to_look[0]
            rospy.loginfo(f"It's your turn, player {player.player_id}!")
            self.look_at_player(player.player_id)
            self.hand_controller.open_hand()
            self.arm_controller.point_at_player(player)
            answer = self.brain_interactor.ask_llm(f'''
tu sei un dungeon master. Il tuo compito è iniziare una nuova avventura. Al momento è il turno di {player.name}, che è un {player.klass}.
I giocatori al tavolo sono:
- {players_to_look[0].name} di classe {players_to_look[0].klass}
- {players_to_look[1].name} di classe {players_to_look[1].klass}

Inventa un inizio di avventura che preveda un'ambientazione in cui si trovano i personaggi. Cerca di inventare qualcosa che sia coinvolgente per i
giocatori e la classe che hanno scelto. Non superare le cinque-sei frasi. Inoltre, non usare segni come asterischi, trattini o underscore. Scrivi il testo
così come lo pronunceresti.
Infine, chiedi al giocatore cosa vuole fare.
''')
            self.brain_interactor.say(answer)

        # players = self.player_db.load_players()
        # self.hand_controller.close_hand()

        # yaws: -0.60, 

        # self.arm_controller.point_at((1.0, 0, 0))

        rospy.loginfo("Player search demonstration complete!")

    def run_face_reco_demo(self, folder, n_faces=6):
        import io
        from PIL import Image
        from deepface import DeepFace
        self.model = DeepFace.build_model("Facenet")
        import torch
        import torch.nn.functional as F

        face_files = [f'{folder}/face_000{i+1}.jpg' for i in range(n_faces)]
        embeddings = []
        for face in face_files:
            embedding_data = DeepFace.represent(face, model_name="Facenet", enforce_detection=False)
            embeddings.append(torch.tensor(embedding_data[0]['embedding']))

        embds = torch.stack(embeddings)  # shape: (n, d)
        X_normalized = F.normalize(embds, p=2, dim=1)
        # Compute similarity via matrix multiplication
        cos_sim_matrix = torch.mm(X_normalized, X_normalized.t())
        print(cos_sim_matrix.shape)
        for i in range(cos_sim_matrix.shape[0]):
            print(cos_sim_matrix[i])


if __name__ == '__main__':
    try:
        node = EnhancedPlayerSearcher(-1.20, 1.20, 0.3)
        node.run_search_demo()
        # node.run_face_reco_demo('.')
    except rospy.ROSInterruptException:
        pass
