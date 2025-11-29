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
from typing import Optional

class MainSceneController:
    """
    Enhanced player search system that implements:
    1. Horizontal head scanning with face centering control
    2. Embedding computation only when faces are properly centered
    3. JSON persistence for player database
    4. Sequential left-to-right player discovery
    """
    
    def __init__(self, prompt: str, first_round_prompt: Optional[str] = None, players_file: str = 'players_database.json', rounds = 3):
        rospy.init_node('enhanced_player_searcher', anonymous=True)
        # Initialize components
        self.head_controller = HeadController()
        self.player_db = PlayerDatabase(players_file)
        self.arm_controller = ArmController()
        self.hand_controller = HandController()
        self.brain_interactor = BrainInteractor(CONFIG_YAML)
        self.prompt = prompt
        self.first_round_prompt = first_round_prompt
        self.players = self.player_db.load_players()
        self.rounds = rounds

        rospy.loginfo(f"Loaded {len(self.players)} players from database")        
        rospy.loginfo("Enhanced Player Searcher initialized")


    def get_prompt(self, context, players, current_player, history):
            history_msg = '\n'.join(history)
            prompt = f'''
    Sei Tiago, un Dungeon Master esperto, descrittivo e imparziale per una campagna di Dungeons & Dragons 5a Edizione.

    # REGOLE FONDAMENTALI (NON INFRANGERLE MAI):
    1. TU SEI L'ARBITRO: Descrivi l'ambiente, gli NPC e le conseguenze delle azioni.
    2. NON GIOCARE PER I GIOCATORI: Non scrivere mai cosa dicono o fanno Marco e Fabiano. Fermati e chiedi loro cosa fanno.
    3. COERENZA: Le tue risposte devono essere logicamente conseguenti all'ultima azione nella <history>.
    4. TIPO DI RISPOSTA: Coinvolgi i giocatori, ma non chiedere direttamente cosa vogliono fare.

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
    Filippo: Credo sia meglio andare a destra
    Matteo: Sono d'accordo, andiamo a destra
    </history>
    Output: Vi ritrovate davanti ad una statua particolare.
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
        if self.first_round_prompt is not None and round_num == 0:
            round_question = self.first_round_prompt
        else:
            round_question = self.brain_interactor.ask_llm(self.get_prompt(context, players, players[0], history))

        self.brain_interactor.say(round_question)
        found_player = self.head_controller.look_at_player(players[0], assert_player_is_here=True)
        if not found_player:
            self.player_not_found(players[0])
            raise Exception("player not found")   
        self.brain_interactor.say(f"{players[0].name}, cosa vuoi fare?")
        p1_ans = self.brain_interactor.hear_str()

        found_player = self.head_controller.look_at_player(players[1], assert_player_is_here=True)
        if not found_player:
            self.player_not_found(players[1])
            raise Exception("player not found")     
        self.brain_interactor.say(f"{players[1].name}, cosa vuoi fare?")
        p2_ans = self.brain_interactor.hear_str()
        # history.append(f"Tiago: {players[0].name}, cosa vuoi fare?")
        history.append(f"{players[0].name}: {p1_ans}")
        # history.append(f"Tiago: {players[1].name}, cosa vuoi fare?")
        history.append(f"{players[1].name}: {p2_ans}")


    
    def run_scene(self):
        self.players = self.player_db.load_players()
        history = [f"Tiago: {self.prompt}"]

        for i in range(self.rounds):
            self.perform_round(self.prompt, history, self.players, i)



if __name__ == '__main__':
    try:
        # node = MainSceneController(
        #     prompt='''
        # Vi trovate all'interno di una taverna.
        # Uno straniero nella taverna proveniente da Mordor si avvicina al tavolo per chiedere se siamo dei cercatori d'oro. Vuole reclutarci per una
        # missione molto pericolosa per sconfiggere il capo degli orchi. Sta cercando delle persone valorose ed è disposto a pagarle bene e ad indicare la strada.
        # ''',
        # first_round_prompt= "Vi trovate all'interno di una taverna. Uno straniero con i vestiti tipici di Mordor si avvicina a voi."
        # )

        node = MainSceneController(
            prompt='''
Vi trovate all'interno alle porte di un antico tempio che è un vero e proprio dungeon con trappole, tesori e varie strade. All'interno è contenuto il tesoro di un vecchio re, che voi siete andati a recuperare. L'ingresso è un grande portone che si apre su una stanza con tante statue e due diverse ramificazioni che portano all'interno del tempio.
            ''',
            first_round_prompt=None
        )
        node.run_scene()

    except rospy.ROSInterruptException:
        pass
