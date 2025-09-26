from typing import Optional
import requests
import pyaudio
import wave
import io

# TODO: read from yaml file
BRAIN_SERVER_URL = "http://localhost:5001"

def ask_llm(text:str):
    url = f"{BRAIN_SERVER_URL}/chat"

    data = [
        {"id": 1, "text": text},
    ]

    response = requests.post(url, json=data)

    return response .json()[0]["response"]


def say(text: str, language: str = "en"):
    url =  f"{BRAIN_SERVER_URL}/tts"
    
    data = [
        {"id": 1, "text": text, "language": language},
    ]

    response = requests.post(url, json=data)
    result = response.json()[0]

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


def transcribe(audio_data, language=None):
    # Send directly to the STT endpoint
    url = f"{BRAIN_SERVER_URL}/stt"
    files = {"file": ("audio.wav", audio_data, "audio/wav")}
    data = {"language": language} if language else {}

    response = requests.post(url, files=files, data=data)
    return response.json()["text"]


if __name__ == "__main__":
# TEST THE FUNCTIONS:
    # Record audio:
    audio_data = hear(duration=5, device_index=None)
    print(f"[DEBUG] Input audio size: {len(audio_data)} bytes")
    # transcribe audio:
    transcription = transcribe(audio_data, language="it")
    print("[DEBUG] Transcription:", transcription)
    # Ask (transcribed) question to llm:
    answer = ask_llm(transcription)
    # Say aloud the answer:
    say(text=answer, language="it")

