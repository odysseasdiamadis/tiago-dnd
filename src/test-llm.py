from transformers import pipeline
from flask import Flask, request, jsonify
import threading
import time
import requests

def create_chat_server(model_name, device=-1):
    app = Flask(__name__)
    generator = pipeline("text-generation", model=model_name, device=device)

    @app.get("/healthy")
    def healthy():
        return "OK"

    @app.post("/")
    def chat():
        out_list = []
        for input in request.json:
            prompt = input['text']
            response = generator(prompt, max_new_tokens=150, do_sample=True, temperature=0.7)[0]['generated_text']
            out_list.append({'id': input['id'], 'response': response})
        return jsonify(out_list)

    return app


def send_test_request():
    time.sleep(5)  # Give the server a moment to start
    test_input = [{'id': 1, 'text': "What is the capital of France?"}]
    try:
        response = requests.post("http://localhost:5001/", json=test_input)
        print("Test Response:")
        print(response.json()[0]["response"])    
    except Exception as e:
        print("Test request failed:", e)

if __name__ == "__main__":
    model_name = "sshleifer/tiny-gpt2"
    app = create_chat_server(model_name, device=-1)

    # Start the test request in a separate thread
    threading.Thread(target=send_test_request, daemon=True).start()

    # Start the server (blocking)
    app.run(host='0.0.0.0', port=5001)