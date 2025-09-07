import os

from transformers import AutoTokenizer, AutoModelForCausalLM, pipeline
import whisper
from gtts import gTTS

from flask import Flask, request, jsonify

# Definition of models used here
# VISION_MODEL = "TODO"
CHAT_MODEL = "teknium/OpenHermes-2.5-Mistral-7B"    # just for quick testing: "sshleifer/tiny-gpt2"
SPEECH_MODEL = "large"                               # just for quick testing: "tiny"
TTS_MODEL = "gtts"

# General params
CACHE_DIR = "src/cache/hugginface/"

class Brain:
    def __init__(self, device=-1):

        # set-up cache 
        os.makedirs(CACHE_DIR, exist_ok=True)
        os.environ["HF_HOME"] = CACHE_DIR    # equivalent to: export HF_HOME="/path/to/cache/dir/"
        os.environ["TRANSFORMERS_CACHE"] = CACHE_DIR

        # set up models
        self.device = device
        self.init_llms()
        
        # boot up server
        self.app = Flask(__name__)
        
        
        @self.app.get("/healthy")
        def healthy():
            return "OK"

        @self.app.post("/chat")
        def chat():
            out_list = []
            for input in request.json:
                prompt = input['text']
                
                # Format prompt for OpenHermes (uses ChatML format)
                formatted_prompt = f"<|im_start|>system\nYou are a helpful AI assistant.<|im_end|>\n<|im_start|>user\n{prompt}<|im_end|>\n<|im_start|>assistant\n"
                
                # Generate response
                response = self.chat_pipeline(
                    formatted_prompt, 
                    max_new_tokens=150, 
                    do_sample=True, 
                    temperature=0.7,
                    pad_token_id=self.chat_tokenizer.pad_token_id,
                    eos_token_id=self.chat_tokenizer.eos_token_id
                )[0]['generated_text']
                
                # Extract only the assistant's response
                if "<|im_start|>assistant" in response:
                    response = response.split("<|im_start|>assistant")[-1].strip()
                    # Remove any end tokens
                    response = response.replace("<|im_end|>", "").strip()
                elif response.startswith(formatted_prompt):
                    response = response[len(formatted_prompt):].strip()
                
                out_list.append({'id': input['id'], 'response': response})
            return jsonify(out_list)


           
        @self.app.post("/tts")
        def text_to_speech():
            out_list = []

            for input in request.json:
                text = input.get('text', '')
                language = input.get('language', 'en')  # fallback to English if not specified

                if not text.strip():
                    out_list.append({'id': input.get('id'), 'error': 'Empty text'})
                    continue

                try:
                    # Check if PulseAudio is in use
                    USE_PULSE = False
                    ps = os.environ.get("PULSE_SERVER")
                    if ps and ps.startswith("unix:"):
                        USE_PULSE = True

                    # Synthesize and save
                    tts = gTTS(text=text, lang=language, slow=False)
                    os.makedirs("audio_output", exist_ok=True)
                    output_path = f"audio_output/{input.get('id', 'output')}.mp3"
                    tts.save(output_path)

                    # Play using mpg123
                    cmd = "mpg123" if not USE_PULSE else "mpg123-pulse"
                    os.system(f"{cmd} {output_path}")

                    out_list.append({'id': input.get('id'), 'status': 'success', 'file': output_path})
                except Exception as e:
                    out_list.append({'id': input.get('id'), 'error': str(e)})

            return jsonify(out_list)

    
        @self.app.route("/stt", methods=["POST"])
        def speech_to_text():
            # Get the audio file from the request
            audio_file = request.files['file']
            
            # Read the audio bytes
            audio_bytes = audio_file.read()
            
            # Save to a temporary path (whisper needs a file path)
            temp_path = "temp_audio.wav"
            with open(temp_path, "wb") as f:
                f.write(audio_bytes)
            
            # Load and process with whisper
            audio = whisper.load_audio(temp_path)
            audio = whisper.pad_or_trim(audio)
            
            # Transcribe
            result = self.speech_2_text_model.transcribe(audio)
            
            # Delete temp file
            import os
            os.remove(temp_path)
            
            return {"text": result["text"]}


        

    def init_llms(self):
        # print(f"Loading model {CHAT_MODEL} into cache at {CACHE_DIR}… this may take a while if first run.")

        self.chat_tokenizer = AutoTokenizer.from_pretrained(
            CHAT_MODEL,
            use_fast=False,          # SentencePiece-based models need slow tokenizer
            cache_dir=CACHE_DIR
        )
        chat_model = AutoModelForCausalLM.from_pretrained(
            CHAT_MODEL,
            cache_dir=CACHE_DIR,
            torch_dtype="auto"
        )

        self.chat_pipeline = pipeline(
            "text-generation",
            model=chat_model,
            tokenizer=self.chat_tokenizer,
            device=self.device
        )

        self.speech_2_text_model = whisper.load_model(SPEECH_MODEL)  


    

    def print_available_routes(self):
        """
        DEBUG: see routes available. Call it after brain server starts to see available routes.
        TODO: additional infos about routes.
        """
        print('='*100)
        print("Brain working. Available Routes:")
        for rule in self.app.url_map.iter_rules():
            if 'POST' in rule.methods:  # print only POST routes (which we will use later)
                print(f"POST -> {rule.rule}")
        print('='*100)


if __name__ == "__main__":
    brain_server = Brain()   
    brain_server.print_available_routes() 
    brain_server.app.run(host="0.0.0.0", port=5001)
