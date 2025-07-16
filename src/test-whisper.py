import whisper
import urllib.request
import os


def download_test_audio(url: str, filename: str):
    if not os.path.exists(filename):
        print(f"Downloading test audio to '{filename}'...")
        urllib.request.urlretrieve(url, filename)
    else:
        print(f"'{filename}' already exists. Skipping download.")
    return filename



def test_whisper(model_name:str):
    # Load the model
    model = whisper.load_model(model_name)  # You can also try "tiny", "small", "medium", or "large"


    # # Download test audio
    # audio_url = "https://cdn.openai.com/whisper/test_audio.mp3"
    # audio_file = download_test_audio(audio_url, "test_audio.mp3")

    # Preprocess audio
    audio_file = whisper.load_audio("audio_output/test_output.mp3")   # ✅ Load audio from file
    audio = whisper.pad_or_trim(audio_file)

    # Make a prediction
    result = model.transcribe(audio)

    return result["text"]

if __name__ == "__main__":

    test_result = test_whisper(model_name="tiny")
    # Print the transcription
    print("Transcription:")
    print(test_result)
