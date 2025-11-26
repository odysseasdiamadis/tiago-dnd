from typing import Optional
import requests
import yaml
import pygame
import pyaudio
import wave
from io import BytesIO
from player_model import Player
import time
import rospy
from std_msgs.msg import UInt8MultiArray
import unicodedata
import re

# file with all setting of the server and interaction modes
CONFIG_YAML = "config/brain_server.yaml"


class BrainInteractor:
    #config:dict     # TODO: define also structure
    brain_url:str

    def __init__(self, yaml_config_path):
        # load config file
        # with open(yaml_config_path, 'r') as f:
        #     self.config = yaml.safe_load(f)
        # define general params
        self.brain_url = self._retrieve_url_from_yaml(yaml_config_path)


    def _retrieve_url_from_yaml(self, yaml_file_path:str):
        # load config file
        with open(yaml_file_path, 'r') as f:
            config = yaml.safe_load(f)

        return f"""{config["SERVER_URL"]}:{config["SERVER_PORT"]}"""


    def ask_llm(self, text:str):
        chat_url = f"{self.brain_url}/chat"

        data = {
            "id": str(time.time()).replace('.',''), 
            "text": text
        }

        response = requests.post(chat_url, json=data)

        # remove special characters and emojis from prompt.
        resp = response.json()["response"]
        rospy.loginfo("LLM RESPONSE: " + resp)
        return resp


    def say(self, text: str, language: str = "it", play_on_server: bool = False):
        tts_url = f"{self.brain_url}/tts"
        sanitized_text = self.sanitize_answer(text)
        data = {
            "text": sanitized_text,
            "language": language,
            "play_on_server": play_on_server
        }

        try:
            response = requests.post(tts_url, json=data, stream=True)
            
            if response.status_code == 200:
                if play_on_server:
                    # Server played the audio
                    result = response.json()
                    #print(f"[TTS] {result.get('status')} - played on {result.get('played_on')}")
                else:
                    # Play on client
                    audio_buffer = BytesIO(response.content)
                    
                    pygame.mixer.init()
                    pygame.mixer.music.load(audio_buffer)
                    pygame.mixer.music.play()
                    
                    # Wait for playback to finish
                    while pygame.mixer.music.get_busy():
                        pygame.time.wait(100)
                        
                    #print("[TTS] Playback completed on client")
            else:
                error = response.json().get('error', 'Unknown error')
                print(f"[TTS] Error: {error}")
                
        except Exception as e:
            print(f"[TTS] Error: {str(e)}")

    def find_mic_index(self, p:pyaudio.PyAudio, auto_select:bool=False) -> int:
        """Tries to automatically find the correct microphone index for the device. 
        Useful in the docker or even if you are not sure.
        @param auto_select: if True, does not ask user to select mic and automatically selects the first non zero one.
        """
        # List devices if device_index is not provided
        print("Available audio input devices:")
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                # if auto_select mode is on, return the first non-zero device_index
                #  - without printing anything
                if auto_select and i != 0:
                    print("Device index: " + str(i))
                    # return i 
                # else, print devices info to select by hand later
                if not auto_select:
                    print(f"  Index {i}: {info['name']}")

        # ask user to select by hand the device id
        device_index = int(input("Enter the device index for your digital mic: "))
        return device_index



    def hear_old(self, duration:int = 5, device_index: Optional[int] = None):
        """Record audio and return it as bytes in WAV format"""
        
        # Recording parameters
        CHUNK = 2048
        FORMAT = pyaudio.paInt32
        CHANNELS = 1    # to correct based on device_index
        RATE = 48000
        RECORD_SECONDS = duration

        # init 
        p = pyaudio.PyAudio()

        # select mic device if it was not provided
        if device_index == None:
            device_index = self.find_mic_index(p, auto_select=False)

        # set param
        device_info = p.get_device_info_by_index(device_index)
        max_channels = device_info['maxInputChannels']
        CHANNELS = min(CHANNELS, max_channels)  # auto-correct request
        
        
    
        # Open stream
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        input_device_index=device_index,
                        frames_per_buffer=CHUNK)
        
        print("Recording...")
        frames = []
        
        for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
        
        print("Recording finished.")
        
        stream.stop_stream()
        stream.close()
        
        # Create WAV file in memory
        wav_buffer = BytesIO()
        wf = wave.open(wav_buffer, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        p.terminate()
        
        # Get the WAV data as bytes
        wav_buffer.seek(0)
        return wav_buffer.read()

    def hear(self, topic_name='/audio_bytes', timeout=30.0):
        try:
            rospy.loginfo(f"In attesa di audio dal topic {topic_name}...")
            
            # Attende UN SINGOLO messaggio (bloccante)
            msg = rospy.wait_for_message(topic_name, UInt8MultiArray, timeout=timeout)
            
            if msg is None:
                raise ValueError("Received NONE from mic")
            # Converte da lista di int a bytes
            audio_bytes = bytes(msg.data)
            
            rospy.loginfo(f"Ricevuto audio: {len(audio_bytes)} bytes")
            return audio_bytes
            
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout o errore ROS: {e}")
            raise

    def sanitize_answer(self, answer: str) -> str:
            # 1. Remove emojis and pictographs
            answer = "".join(
                ch for ch in answer 
                if not unicodedata.category(ch).startswith("So")  # Symbol, other
                and not unicodedata.category(ch).startswith("Sk")  # Symbol, modifier
                and not unicodedata.category(ch).startswith("Cs")  # Surrogates (emoji)
            )
            
            # 2. Remove other “noisy” symbols
            # Customize this list as needed
            remove_chars = r"[*_~`^#@|<>\\{}\[\]/+=%$€£¥]"
            answer = re.sub(remove_chars, "", answer)

            # 3. Replace multiple punctuation marks with a single one
            answer = re.sub(r"[!?]{2,}", lambda m: m.group(0)[0], answer)
            answer = re.sub(r"[.]{2,}", ".", answer)

            # 4. Remove stray standalone punctuation
            # answer = re.sub(r"\s+[!?.,;:]\s+", " ", answer)

            # 5. Normalize spaces
            answer = re.sub(r"\s+", " ", answer).strip()

            return answer

    def transcribe(self, audio_data, language=None):
        # Send directly to the STT endpoint
        stt_url = f"{self.brain_url}/stt"
        files = {"file": ("audio.wav", audio_data, "audio/wav")}
        data = {"language": language} if language else {}
        data["id"] = str(time.time()).replace('.','')

        response = requests.post(stt_url, files=files, data=data)
        return response.json()["text"]

    def hear_str(self):
        audio_data = self.hear()
        return self.transcribe(audio_data)
    

    def ask_player_name_and_class(self):
        data = self.hear()
        player_answer = self.transcribe(data, language='it')
        prompt = f'''
Il tuo compito è classificare una risposta di un giocatore per ottenere nome e classe per un'avventura in stile Dungeons and Dragons.
Il giocatore ha risposto alla domanda "come ti chiami e qual è la tua classe?".
Devi rispondere con un'unica stringa JSON nel formato {{"name": <il nome del giocatore>, "class": <la classe di avventuriero che ha scelto> }}.
Non devi produrre altro output oltre la stringa json richiesta.

Esempio: "Mi chiamo Andrea e mi è sempre piaciuto usare il paladino"
{{ "name": "Andrea", "class": "paladino" }}

La risposta del giocatore è: {player_answer}
        '''
        answer = self.ask_llm(prompt)
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

if __name__ == "__main__":
# TEST THE FUNCTIONS:

    # create interactor obj
    # Say aloud the answer:
    # interactor.say(text=answer, language="it")
    #test_new_player_with_ammonitions()
   
    interactor = BrainInteractor(CONFIG_YAML)
    interactor.hear()
