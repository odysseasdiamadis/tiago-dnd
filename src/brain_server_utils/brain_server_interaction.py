from typing import Optional
import requests
import yaml
import pygame
import pyaudio
import wave
from io import BytesIO

import time

# file with all setting of the server and interaction modes
CONFIG_YAML = "src/config/brain_server.yaml"


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

        return response.json()["response"]


    def say(self, text: str, language: str = "en", play_on_server: bool = False):
        tts_url = f"{self.brain_url}/tts"
        
        data = {
            "text": text,
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
                    return i 
                # else, print devices info to select by hand later
                if not auto_select:
                    print(f"  Index {i}: {info['name']}")

        # ask user to select by hand the device id
        device_index = int(input("Enter the device index for your digital mic: "))
        return device_index



    def hear(self, duration:int = 5, device_index: Optional[int] = None):
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
            device_index = self.find_mic_index(p, auto_select=True)

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


    def transcribe(self, audio_data, language=None):
        # Send directly to the STT endpoint
        stt_url = f"{self.brain_url}/stt"
        files = {"file": ("audio.wav", audio_data, "audio/wav")}
        data = {"language": language} if language else {}
        data["id"] = str(time.time()).replace('.','')

        response = requests.post(stt_url, files=files, data=data)
        return response.json()["text"]


if __name__ == "__main__":
# TEST THE FUNCTIONS:

    # create interactor obj
    interactor = BrainInteractor(CONFIG_YAML)

    # Record audio:
    audio_data = interactor.hear(duration=5, device_index=None)
    print(f"[DEBUG] Input audio size: {len(audio_data)} bytes")
    # transcribe audio:
    transcription = interactor.transcribe(audio_data, language="it")
    print("[DEBUG] Transcription:", transcription)
    # Ask (transcribed) question to llm:
    answer = interactor.ask_llm(transcription)
    print("[DEBUG] LLM answer:", answer)
    # Say aloud the answer:
    interactor.say(text=answer, language="it")

