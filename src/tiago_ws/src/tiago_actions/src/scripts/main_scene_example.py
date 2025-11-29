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

class MainSceneController:
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
        
        rospy.loginfo("Enhanced Player Searcher initialized")


    def get_prompt(self, context, players, current_player, history):
            history_msg = '\n'.join(history)
            prompt = f'''
    Sei Tiago, un Dungeon Master esperto, descrittivo e imparziale per una campagna di Dungeons & Dragons 5a Edizione.

    # REGOLE FONDAMENTALI (NON INFRANGERLE MAI):
    1. TU SEI L'ARBITRO: Descrivi l'ambiente, gli NPC e le conseguenze delle azioni.
    2. NON GIOCARE PER I GIOCATORI: Non scrivere mai cosa dicono o fanno Marco e Fabiano. Fermati e chiedi loro cosa fanno.
    3. COERENZA: Le tue risposte devono essere logicamente conseguenti all'ultima azione nella <history>.

    # I GIOCATORI:
    - {players[0].name} di classe {self.players[0].klass}
    - {players[1].name} di classe {self.players[1].klass}

    # STATO ATTUALE
    <context>
    f{context}
    </context>

    # STORIA DELLA CONVERSAZIONE:
    <history>
    {history_msg}
    Tiago:
    </history>

    # ISTRUZIONI PER LA RISPOSTA:
    Analizza l'ultima azione dei giocatori nella history.
    Prosegui la narrazione con max 3 frasi.
    Quando è presente un NPC nella scena e i player vi interagiscono,
    fai parlare e muovi l'NPC in rapporto alle azioni dei giocatori.
    Non devi **MAI** agire per conto dei giocatori, ma devi includere quello che dicono
    o fanno nella scena.

    <example>
    <context>
    I giocatori si trovano in un dungeon pieno di mostri e sono davanti a un bivio.
    </context>

    # UN ESEMPIO COME RIFERIMENTO
    I giocatori sono Filippo e Matteo. È il turno di Matteo.
    <history>
    Tiago: Filippo, cosa vuoi fare?
    Filippo: Credo sia meglio andare a destra
    Tiago: Matteo, cosa vuoi fare?
    Matteo: Sono d'accordo, andiamo a destra
    </history>
    Vi ritrovate davanti ad una statua particolare.
    </example>
    '''
            return prompt

    def player_not_found(self, player):
        prompt = f'''Sei un dungeon master di DnD. Ti sei appena accorto che un giocatore ha lasciato il tavolo. Produci un messaggio
        per chiedergli dove sia ed invitarlo a giocare, dato che senza di lui non si può continuare. Il nome del giocatore è {player.name}.
        Devi produrre solo e soltanto la risposta del DM. Produci una risposta breve e concisa.'''
        msg = self.brain_interactor.ask_llm(prompt)
        self.brain_interactor.say(msg)
    
    def perform_round(self, context, history, players, round_num):
        if round_num == 0:
            round_question = "Vi trovate all'interno di una taverna. Uno straniero con i vestiti tipici di Mordor si avvicina a voi."
        else:
            round_question = self.brain_interactor.ask_llm(self.get_prompt(context, players, players[0], history))

        self.brain_interactor.say(round_question)

        for i, player in enumerate(players):
            found_player = self.head_controller.look_at_player(player)
            if not found_player:
                self.player_not_found(player)
                raise Exception(f"Player {player.name} not found!")
            self.arm_controller.point_at_player(player)
            tiago_q = f"{player.name}, cosa vuoi fare?"
            self.brain_interactor.say(tiago_q)
            p1_ans = self.brain_interactor.hear_str()
            history.append(tiago_q)
            history.append(p1_ans)

    
    def run_scene(self):
        self.context_prompt = '''
        Vi trovate all'interno di una taverna.
        Uno straniero nella taverna proveniente da Mordor si avvicina al tavolo per chiedere se siamo dei cercatori d'oro. Vuole reclutarci per una
        missione molto pericolosa per sconfiggere il capo degli orchi. Sta cercando delle persone valorose ed è disposto a pagarle bene e ad indicare la strada.
        '''

        self.players = self.player_db.load_players()
        history = [f"Tiago: {self.context_prompt}"]

        for i in range(5):
            self.perform_round(self.context_prompt, history, self.players, i)



if __name__ == '__main__':
    try:
        node = MainSceneController(-1.20, 1.20, 0.3)
        node.run_scene()
        # players = node.player_db.load_players()
        # can_find_player = node.head_controller.look_at_player(players[0])
        # if not can_find_player:
        #     node.brain_interactor.say(f"Ei {players[0].name}, dove sei?")
        # can_find_player = node.head_controller.look_at_player(players[1])
        # if not can_find_player:
        #     node.brain_interactor.say(f"Ei {players[1].name}, dove sei?")

    except rospy.ROSInterruptException:
        pass
