import os
from io import BytesIO
import yaml

import torch
# from transformers import AutoTokenizer, AutoModelForCausalLM, pipeline
import whisper
from gtts import gTTS

import pygame
from flask import Flask, request, jsonify, send_file

from brain_server_utils.llm_preproc_utils import GPT2, Qwen3, Mistral

# file with all setting of the server and interaction modes
CONFIG_YAML = "src/config/brain_server.yaml"


class Brain:
    def __init__(self, config_yaml_file:str, device=-1):
        assert os.path.isfile(config_yaml_file), f"WARNING: tiago_server.yaml file not found, refer to src/config/brain_server.example.yaml to create one."
        
        # load config file
        with open(config_yaml_file, 'r') as f:
            self.config = yaml.safe_load(f)


        # set-up cache 
        os.makedirs(self.config["CACHE_DIR"], exist_ok=True)
        os.environ["HF_HOME"] = self.config["CACHE_DIR"]    # equivalent to: export HF_HOME="/path/to/cache/dir/"
        os.environ["TRANSFORMERS_CACHE"] = self.config["CACHE_DIR"]

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
            input_data = request.json or {}
            prompt = input_data.get('text')
            
            # Generate response directly through GPT2 class
            response = self.chat_model.chat(
                prompt
                # max_new_tokens=250,
                # temperature=0.7
            )
            
            return jsonify({
                'id': request.json.get('id'),
                'response': response
            })

           

        @self.app.post("/tts")
        def text_to_speech():
            input_data = request.json or {}
            text = input_data.get('text', '')
            language = input_data.get('language', 'en')
            play_on_server = input_data.get('play_on_server', False)  # Toggle option

            if not text.strip():
                return jsonify({'error': 'Empty text'}), 400

            try:
                # Generate audio in memory
                tts = gTTS(text=text, lang=language, slow=False)
                audio_buffer = BytesIO()
                tts.write_to_fp(audio_buffer)
                audio_buffer.seek(0)
                
                if play_on_server:
                    # Play on server
                    pygame.mixer.init()
                    pygame.mixer.music.load(audio_buffer)
                    pygame.mixer.music.play()
                    
                    # Wait for playback to finish
                    while pygame.mixer.music.get_busy():
                        pygame.time.wait(100)
                    
                    return jsonify({'status': 'success', 'played_on': 'server'})
                else:
                    # Stream the audio to client
                    audio_buffer.seek(0)  # reset buffer position
                    return send_file(
                        audio_buffer,
                        mimetype='audio/mpeg',
                        as_attachment=False,
                        download_name='speech.mp3'
                    )
                
            except Exception as e:
                return jsonify({'error': str(e)}), 500
            

            
        @self.app.route("/stt", methods=["POST"])
        def speech_to_text():
            # Get the audio file and params from the request
            audio_file = request.files['file']
            language = request.form.get('language', "")  

            # Read the audio bytes
            audio_bytes = audio_file.read()

            #Save to a temporary path (whisper needs a file path)
            temp_path = "temp_audio.wav"
            with open(temp_path, "wb") as f:
                f.write(audio_bytes)

            # Load and process with whisper
            audio = whisper.load_audio(temp_path)
            audio = whisper.pad_or_trim(audio)

            # Transcribe with or without language
            if language:
                result = self.speech_2_text_model.transcribe(audio, language=language)
            else:
                result = self.speech_2_text_model.transcribe(audio)

            #Deletes temp file
            os.remove(temp_path)

            return jsonify({
                "id": request.form.get("id", ""),
                "text": result["text"]
            })



        

    def init_llms(self):
 
        # Load one of the supported models
        print("[NOTE] Known supported models: teknium/OpenHermes-2.5-Mistral-7B, sshleifer/tiny-gpt2, Qwen/Qwen3-4B-Instruct-2507")
        if "GPT-2" in self.config["CHAT_MODEL"]:
            print("[DEBUG] Trying to load a GPT-2 based model...")
            self.chat_model = GPT2(self.config)
        elif "Qwen3" in self.config["CHAT_MODEL"]:
            print("[DEBUG] Trying to load a Qwen3 based model...")
            self.chat_model = Qwen3(self.config)
        elif "Mistral" in self.config["CHAT_MODEL"]:
            print("[DEBUG] Trying to load a Mistral based model...")
            self.chat_model = Mistral(self.config)
        else:
            raise ValueError(f"""Impossible to load model: {self.config["CHAT_MODEL"]}""")

        self.speech_2_text_model = whisper.load_model(self.config["SPEECH_MODEL"])  


    

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
    brain_server = Brain(
        config_yaml_file=CONFIG_YAML,
        device=0 if torch.cuda.is_available() else -1       # select first gpu available or cpu if gpu is not available
    )   
    brain_server.print_available_routes() 
    brain_server.app.run(host="0.0.0.0", port=5001)

    # NOTE: On the same pc you run this script, you can grep the current ip on which this service will be exposed:
    #        ifconfig | grep 'inet ' | grep -v '127.0.0.1' | awk '{print $2}'
    #       This returns the ip that you will also see when starting the brain server and you can add the port.
    #       To reach the service from other pcs connected to the same network, you also need to enable the requests through the firewall.
    #       On windows, run:
    #       New-NetFirewallRule -DisplayName "Brain Server" -Direction Inbound -Protocol TCP -LocalPort 5001 -Action Allow
    #       On linux, run:
            # TODO
