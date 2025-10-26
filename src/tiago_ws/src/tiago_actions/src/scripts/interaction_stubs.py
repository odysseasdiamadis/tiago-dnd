from typing import Optional
import time
from player_model import Player

"""
WARNING: this file has the same function names og brain_server_interaction.
These functions DO NOTHING and are here just to test some logic without starting docker or brain server.
DO NOT USE THEM FOR ACTUAL SIMULATION.
"""

CONFIG_YAML = "test_path/file.yaml"

class BrainInteractor:
    brain_url: str

    def __init__(self, yaml_config_path):
        print(f"[STUB] Initializing BrainInteractor with config: {yaml_config_path}")
        time.sleep(1)
        self.brain_url = self._retrieve_url_from_yaml(yaml_config_path)

    def _retrieve_url_from_yaml(self, yaml_file_path: str):
        print(f"[STUB] Retrieving URL from YAML file: {yaml_file_path}")
        time.sleep(1)
        return "http://localhost:8000"

    def ask_llm(self, text: str):
        print(f"[STUB] Asking LLM with text: {text[:50]}...")
        time.sleep(1)
        return "This is a stub response from the LLM"

    def say(self, text: str, language: str = "it", play_on_server: bool = False):
        print(f"[STUB] Speaking text: '{text[:50]}...' in language: {language}, play_on_server: {play_on_server}")
        time.sleep(1)

    def find_mic_index(self, p=None, auto_select: bool = False) -> int:
        print(f"[STUB] Finding microphone index, auto_select: {auto_select}")
        time.sleep(1)
        return 1

    def hear(self, duration: int = 5, device_index: Optional[int] = None):
        print(f"[STUB] Recording audio for {duration} seconds from device index: {device_index}")
        time.sleep(1)
        return b"stub_audio_data_bytes"

    def transcribe(self, audio_data, language=None):
        print(f"[STUB] Transcribing audio data of size {len(audio_data)} bytes, language: {language}")
        time.sleep(1)
        return "This is a stub transcription"

    def ask_player_name_and_class(self):
        print("[STUB] Asking player for name and class")
        time.sleep(1)
        return '{"name": "TestPlayer", "class": "warrior"}'

    def welcome_new_player(self, player: Player):
        print(f"[STUB] Welcoming new player: {player.name}, class: {player.klass}, ammonitions: {player.ammonitions}")
        time.sleep(1)
