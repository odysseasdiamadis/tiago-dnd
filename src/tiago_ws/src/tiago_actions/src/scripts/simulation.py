from typing import Optional
import requests
import yaml
import pygame
import pyaudio
import wave
from io import BytesIO
import threading
import time


# dummy functions to use without tiago/brain server: (DO NOT USE FOR ACTUAL SIMULATION)
# from interaction_stubs import *     # only suited for tests
from brain_server_interaction import *

from player_model import Player
from button_interaction_utils import * 



def ask_player_name_and_class(interactor:BrainInteractor):
    data = interactor.hear()
    player_answer = interactor.transcribe(data, language='it')
    prompt = f'''
Il tuo compito è classificare una risposta di un giocatore per ottenere nome e classe per un'avventura in stile Dungeons and Dragons.
Il giocatore ha risposto alla domanda "come ti chiami e qual è la tua classe?".
Devi rispondere con un'unica stringa JSON nel formato {{"name": <il nome del giocatore>, "class": <la classe di avventuriero che ha scelto> }}.
Non devi produrre altro output oltre la stringa json richiesta.

Esempio: "Mi chiamo Andrea e mi è sempre piaciuto usare il paladino"
{{ "name": "Andrea", "class": "paladino" }}

La risposta del giocatore è: {player_answer}
    '''
    answer = interactor.ask_llm(prompt)
    return answer

def welcome_new_player(self, player: Player):
    player_data = {
        "name": player.name,
        "class": player.klass,
        "ammonitions": player.ammonitions,
    }
    prompt = f'''
Tu sei un dungeon master e il tuo compito è accogliere un giocatore che si è appena seduto al tavolo.
Ti verrà passato un JSON con la seguente struttura:
{{"name": <nome del giocatore>, "class": <la classe che ha scelto o giocato l'ultima volta>, "ammonitions": <il numero di volte che è stato richiamato per cattiva condotta>}}
Un giocatore può avere un'ammonizione se si è comportato male, magari insultando un altro giocatore o barando coi dadi o usando un linguaggio non appropriato.

Il tuo compito è rispondere con una frase per accoglierlo e nel caso abbia ammonizioni ricordargli di mantenere una buona condotta.
Rispondi solo e unicamente con la frase di benvenuto, non aggiungere nient'altro.

Il giocatore è: {player_data}
'''
    welcome_msg = self.ask_llm(prompt)
    self.say(welcome_msg, language='it')

def test_new_player():
    interactor = BrainInteractor(CONFIG_YAML)

    # Record audio:
    audio_data = interactor.hear(duration=5, device_index=None)
    print(f"[DEBUG] Input audio size: {len(audio_data)} bytes")
    # transcribe audio:
    transcription = interactor.transcribe(audio_data, language="it")
    print("[DEBUG] Transcription:", transcription)
    # Ask (transcribed) question to llm:
    # answer = interactor.ask_llm(transcription)
    answer = interactor.parse_player_name_and_class(transcription)
    print("[DEBUG] LLM answer:", answer)


def test_new_player_with_ammonitions():
    interactor = BrainInteractor(CONFIG_YAML)
    player = {
        "name": "Marco",
        "class": "guerriero",
        "ammonitions": 1,
    }

    player2 = {
        "name": "Alice",
        "class": "paladino",
        "ammonitions": 0,
    }

    prompt = '''
Tu sei un dungeon master e il tuo compito è accogliere un giocatore che si è appena seduto al tavolo.
Ti verrà passato un JSON con la seguente struttura:
{{"name": <nome del giocatore>, "class": <la classe che ha scelto o giocato l'ultima volta>, "ammonitions": <il numero di volte che è stato richiamato per cattiva condotta>}}
Un giocatore può avere un'ammonizione se si è comportato male, magari insultando un altro giocatore o barando coi dadi o usando un linguaggio non appropriato.

Il tuo compito è rispondere con una frase per accoglierlo e nel caso abbia ammonizioni ricordargli di mantenere una buona condotta.
Rispondi solo e unicamente con la frase di benvenuto, non aggiungere nient'altro.

Il giocatore è: {player}
    '''

    print(interactor.ask_llm(prompt.format(player=player)))
    print(interactor.ask_llm(prompt.format(player=player2)))


class GameSession:
    def __init__(self, config_yaml):
        self.interactor = BrainInteractor(config_yaml)
        self.player = None
        self.state = "WAITING_FOR_PLAYER"  # States: WAITING_FOR_PLAYER, PLAYING
        self.interaction_gui = None
        self.recorder = None
        
    def startup(self):
        """Initialize the game session"""
        print("Starting D&D Game Session...")
        # TODO: search players and init tiago 
        
    def handle_audio(self, audio_bytes):
        """Route audio handling based on current state"""
        if self.state == "WAITING_FOR_PLAYER":
            self.handle_initial_recording(audio_bytes)
        elif self.state == "PLAYING":
            self.handle_game_turn(audio_bytes)
    
    def handle_initial_recording(self, audio_bytes):
        """First button press: get player name and class"""
        print(f"Setting up player... ({len(audio_bytes)} bytes)")
        # TODO: implement logic of setting up player by asking class etc + welcoming.
        # Possible example of flow below, to decide

        # transcription = self.interactor.transcribe(audio_bytes, language="it")
        # player_data = ask_player_name_and_class(self.interactor)  # Your existing function
        
        # # Create player
        # self.player = Player(name=player_data['name'], klass=player_data['class'])
        
        # # Welcome player
        # welcome_new_player(self.interactor, self.player)

        # + add setup game scenario
        
        # Transition to playing state
        self.state = "PLAYING"
        print("Player setup complete! Game started - press and release button to record each turn of the conversation")
    
    def handle_game_turn(self, audio_bytes):
        """Subsequent button presses: handle game interactions"""
        print(f"Processing turn... ({len(audio_bytes)} bytes)")
        # TODO: logic here for each turn of the conversation.
        # Ideally, it should be: player records -> ask llm based on info -> llm responds and updates some status
        
        # # Transcribe player input
        # transcription = self.interactor.transcribe(audio_bytes, language="it")
        # print(f"Player said: {transcription}")
        
        # # game logic here:
        # prompt = f"The player says: {transcription}. Respond as a D&D dungeon master."
        # response = self.interactor.ask_llm(prompt)
        
        # # Respond
        # self.interactor.say(response, language="it")
        print("Turn complete! Press press and release to record for next turn")
    
    def run(self):
        """Main entry point"""
        self.startup()
        
        # Create button/GUI with audio callback
        self.interaction_gui, self.recorder = create_record_window(
            on_audio_ready=self.handle_audio
        )
        
        print("Press the button to introduce yourself (name and class)...")
        # TODO: this should be done by tiago, but can be said just after creating button mainloop
        
        # Start GUI event loop (blocking)
        self.interaction_gui.mainloop()


if __name__ == "__main__":
    game = GameSession(CONFIG_YAML)
    game.run()