from typing import Optional
import requests
import yaml
import pyaudio
import wave
import io

# file with all setting of the server and interaction modes
CONFIG_YAML = "src/config/brain_server.yaml"


def retrieve_url_from_yaml(yaml_file_path:str):
    # load config file
    with open(yaml_file_path, 'r') as f:
        config = yaml.safe_load(f)

    return f"""{config["SERVER_URL"]}:{config["SERVER_PORT"]}"""


def ask_llm(server_url:str, text:str):
    chat_url = f"{server_url}/chat"

    data = [
        {"id": 1, "text": text},
    ]

    response = requests.post(chat_url, json=data)

    return response.json()[0]["response"]


def say(server_url:str, text: str, language: str = "en"):
    tts_url =  f"{server_url}/tts"
    
    data = [
        {"id": 1, "text": text, "language": language},
    ]

    response = requests.post(tts_url, json=data)
    result = response.json()[0]

    # TODO: check audio saving in file, is it necessary?
    # TODO: does the SERVER play audio OR does this function do? This can cause problems if the server is hosted on another pc!!
    if "status" in result and result["status"] == "success":
        print(f"[TTS] Audio saved to: {result['file']}")
    else:
        print(f"[TTS] Error: {result.get('error', 'Unknown error')}")


def find_mic_index(p:pyaudio.PyAudio, auto_select:bool=False) -> int:
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



def hear(duration:int = 5, device_index: Optional[int] = None):
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
        device_index = find_mic_index(p, auto_select=True)

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
    wav_buffer = io.BytesIO()
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


def transcribe(server_url:str, audio_data, language=None):
    # Send directly to the STT endpoint
    stt_url = f"{server_url}/stt"
    files = {"file": ("audio.wav", audio_data, "audio/wav")}
    data = {"language": language} if language else {}

    response = requests.post(stt_url, files=files, data=data)
    return response.json()["text"]


if __name__ == "__main__":
# TEST THE FUNCTIONS:
    # get base urk of brain server
    server_url = retrieve_url_from_yaml(CONFIG_YAML)
    # Record audio:
    audio_data = hear(duration=5, device_index=None)
    print(f"[DEBUG] Input audio size: {len(audio_data)} bytes")
    # transcribe audio:
    transcription = transcribe(server_url, audio_data, language="it")
    print("[DEBUG] Transcription:", transcription)
    # Ask (transcribed) question to llm:
    answer = ask_llm(server_url, transcription)
    # Say aloud the answer:
    say(server_url, text=answer, language="it")

