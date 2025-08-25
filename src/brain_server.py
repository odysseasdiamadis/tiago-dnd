import os

from transformers import pipeline
from flask import Flask, request, jsonify

# Definition of models used here
VISION_MODEL = "TODO"
CHAT_MODEL = "microsoft/phi-2"               # just for quick testing: "sshleifer/tiny-gpt2"
SPEECH_MODEL = "tiny"
# TTS_MODEL = "google_tts"

# General params
CACHE_DIR = "src/cache/hugginface/"

class Brain:
    def __init__(self, device=-1):

        # set-up cache 
        os.makedirs(CACHE_DIR, exist_ok=True)
        os.environ["HF_HOME"] = CACHE_DIR    # equivalent to: export HF_HOME="/path/to/cache/dir/"
        os.environ["TRANSFORMERS_CACHE"] = CACHE_DIR
        
        # boot up server
        self.app = Flask(__name__)
        self.chat_pipeline = pipeline("text-generation", model=CHAT_MODEL, device=device)

        
        @self.app.get("/healthy")
        def healthy():
            return "OK"

        @self.app.post("/chat")
        def chat():
            out_list = []
            for input in request.json:
                prompt = input['text']
                response = self.chat_pipeline(prompt, max_new_tokens=150, do_sample=True, temperature=0.7)[0]['generated_text']
                out_list.append({'id': input['id'], 'response': response})
            return jsonify(out_list)
        
    def init_llms():
        pass
    

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
