#!/usr/bin/env python

from dataclasses import dataclass
from typing import List, Dict
from datetime import datetime
import json
import numpy as np

@dataclass
class Player:
    """Represents a detected player with face embedding and position information."""
    face_embedding: List[float]
    yaw: float
    face_position: tuple
    is_present: bool
    player_id: int
    discovered_time: str = None
    ammonitions: int = 0
    
    def __post_init__(self):
        if self.discovered_time is None:
            self.discovered_time = datetime.now().isoformat()
    
    def to_dict(self) -> Dict:
        """Convert player to dictionary for JSON serialization."""
        return {
            'face_embedding': self.face_embedding if isinstance(self.face_embedding, list) else self.face_embedding.tolist(),
            'yaw': float(self.yaw),
            'face_position': [float(x) for x in self.face_position],
            'player_id': int(self.player_id),
            'discovered_time': self.discovered_time
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Player':
        """Create player from dictionary."""
        p = cls(
            face_embedding=data['face_embedding'],
            yaw=data['yaw'], 
            face_position=tuple(data['face_position']),
            player_id=data['player_id'],
            is_present=data['is_present'],
            discovered_time=data['discovered_time'],
        )
        p.is_present = False
        return p
    
    def get_face_center_x(self) -> float:
        """Get the x-coordinate of face center."""
        return (self.face_position[0] + self.face_position[2]) / 2
    
    def get_face_embedding_array(self) -> np.ndarray:
        """Get face embedding as numpy array."""
        return np.array(self.face_embedding)


class PlayerDatabase:
    """Manages persistence and retrieval of player data."""
    
    def __init__(self, database_file: str = 'players_database.json'):
        self.database_file = database_file
        self.players = []
    
    def load_players(self) -> List[Player]:
        """Load players from JSON file."""
        try:
            with open(self.database_file, 'r') as f:
                data = json.load(f)
                self.players = [Player.from_dict(player_data) for player_data in data.get('players', [])]

                return self.players
        except (FileNotFoundError, json.JSONDecodeError):
            self.players = []
            return self.players
    
    def save_players(self, players: List[Player]) -> None:
        """Save players to JSON file."""
        data = {
            'players': [player.to_dict() for player in players],
            'last_updated': datetime.now().isoformat(),
            'total_players': len(players)
        }
        with open(self.database_file, 'w') as f:
            json.dump(data, f, indent=2)
    
    def add_player(self, player: Player) -> None:
        """Add a new player to the database."""
        self.players.append(player)
        # self.save_players(self.players)
    
    def update_player(self, player_id: int, updated_player: Player) -> bool:
        """Update an existing player in the database."""
        for i, player in enumerate(self.players):
            if player.player_id == player_id:
                self.players[i] = updated_player
                self.save_players(self.players)
                return True
        return False
    
    def get_next_player_id(self) -> int:
        """Get the next available player ID."""
        if not self.players:
            return 0
        return max(player.player_id for player in self.players) + 1