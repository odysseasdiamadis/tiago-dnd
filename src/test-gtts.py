from gtts import gTTS
import os

# text = "This is a test of Google Text-to-Speech on Linux."
# language = 'en'

USE_PULSE = False
ps = os.environ.get("PULSE_SERVER")
if ps and ps.startswith("unix:"):
    USE_PULSE = True

text = "Ciao, il mio nome è Tiago!"
language = 'it'

tts = gTTS(text=text, lang=language, slow=False)
os.makedirs("audio_output", exist_ok=True)
tts.save("audio_output/test_output.mp3")

# Play using mpg123
cmd = "mpg123" if not USE_PULSE else "mpg-pulse"
os.system(f"{cmd} audio_output/test_output.mp3")
