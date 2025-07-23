import whisper
import urllib.request
import os

import pyaudio
import wave


def download_test_audio(url: str, filename: str):
    if not os.path.exists(filename):
        print(f"Downloading test audio to '{filename}'...")
        urllib.request.urlretrieve(url, filename)
    else:
        print(f"'{filename}' already exists. Skipping download.")
    return filename



def test_whisper(model_name:str, audio_sample_path:str):
    # Load the model
    model = whisper.load_model(model_name)  # You can also try "tiny", "small", "medium", or "large"


    # # Download test audio
    # audio_url = "https://cdn.openai.com/whisper/test_audio.mp3"
    # audio_file = download_test_audio(audio_url, "test_audio.mp3")

    # Preprocess audio
    audio_file = whisper.load_audio(AUDIO_SAMPLE)   
    audio = whisper.pad_or_trim(audio_file)

    # Make a prediction
    result = model.transcribe(audio)

    return result["text"]

def test_mic_input(audio_output_path:str):
    """
    @param audio_output_path: where to save the mic input got by this func.
    """
    # Recording parameters
    CHUNK = 1024  # Number of frames to read at a time
    FORMAT = pyaudio.paInt16  # 16-bit integer format
    CHANNELS = 2  # Number of audio channels (1 for mono, 2 for stereo)
    RATE = 44100  # Sample rate (Hz)
    RECORD_SECONDS = 5  # Duration of recording (seconds)

    # Create PyAudio object
    p = pyaudio.PyAudio()

    # Open stream
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    print("Recording...")
    frames = []

    # Record audio
    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("Recording finished.")

    # Close and terminate everything properly
    stream.stop_stream()
    stream.close()
    p.terminate()

    # Save as WAV file
    os.makedirs(os.path.dirname(audio_output_path), exist_ok=True)    
    wf = wave.open(audio_output_path, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    print(f"Audio saved as {audio_output_path}")

if __name__ == "__main__":

    WHISPER_MODEL="tiny"
    AUDIO_SAMPLE="audio_output/test_mic.wav"

    #test_mic_input(audio_output_path=AUDIO_SAMPLE)

    test_result = test_whisper(model_name=WHISPER_MODEL, audio_sample_path=AUDIO_SAMPLE)

    # Print the transcription
    print("Transcription:")
    print(test_result)
