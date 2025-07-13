from gtts import gTTS
import os

# text = "This is a test of Google Text-to-Speech on Linux."
# language = 'en'

text = "Ciao, il mio nome è Tiago!"
language = 'it'

tts = gTTS(text=text, lang=language, slow=False)
os.makedirs("audio_output", exist_ok=True)
tts.save("audio_output/test_output.mp3")

# Play using mpg123
os.system("mpg123 audio_output/test_output.mp3")
