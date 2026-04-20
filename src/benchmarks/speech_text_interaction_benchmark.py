"""
TEST: READ FROM FILE -> TTS -> LISTEN -> TRASCRIBE
"""

from typing import Optional
import requests
import pyaudio
import wave
import io

import tkinter as tk
from tkinter import font


def ask_llm(text:str):
    url = "http://localhost:5001/chat"

    data = [
        {"id": 1, "text": text},
    ]

    response = requests.post(url, json=data)

    return response .json()[0]["response"]


def say(text: str, language: str = "en"):
    url = "http://localhost:5001/tts"

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
            # if auto_select mode is on, return the first non-zero device_index without printing anything
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
    url = "http://localhost:5001/stt"
    files = {"file": ("audio.wav", audio_data, "audio/wav")}
    data = {"language": language} if language else {}

    response = requests.post(url, files=files, data=data)
    return response.json()["text"]



class AudioRecorder:
    def __init__(self, master, rate=48000, chunk=2048, format=pyaudio.paInt16):
        self.master = master   # Tk root object
        self.rate = rate
        self.chunk = chunk
        self.format = format
        self.channels = 1
        self.device_index = None

        self.p = pyaudio.PyAudio()
        self.stream = None
        self.frames = []
        self.recording = False
        self.after_id = None

    def start_recording(self, device_index: Optional[int] = None):
        if device_index is None:
            device_index = self.find_mic_index(auto_select=True)

        device_info = self.p.get_device_info_by_index(device_index)
        self.channels = min(self.channels, device_info['maxInputChannels'])

        self.stream = self.p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=self.chunk
        )

        self.frames = []
        self.recording = True
        print("Recording start")

        # Start reading chunks via Tkinter .after()
        self._record_step()

    def stop_recording(self) -> bytes:
        self.recording = False
        if self.after_id:
            self.master.after_cancel(self.after_id)
            self.after_id = None

        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None

        print("Recording stopped")

        # Save WAV into memory
        buf = io.BytesIO()
        wf = wave.open(buf, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(self.frames))
        wf.close()
        buf.seek(0)
        return buf.read()

    def _record_step(self):
        """Read one audio chunk and reschedule if still recording"""
        if self.recording and self.stream:
            try:
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                self.frames.append(data)
                # Schedule next read in a few ms
                self.after_id = self.master.after(10, self._record_step)
            except Exception as e:
                print(f"Error reading audio: {e}")
                self.recording = False

    def terminate(self):
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()

    def find_mic_index(self,  auto_select:bool=False) -> int:
        """Tries to automatically find the correct microphone index for the device. 
        Useful in the docker or even if you are not sure.
        @param auto_select: if True, does not ask user to select mic and automatically selects the first non zero one.
        NOTE: this is a modified version of the above unit tests one, with slight modifications to make it work in a class.
        """
        # List devices if device_index is not provided
        print("Available audio input devices:")
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
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


def create_record_window():
    """Creates the UI to send a vocal message and interact with an llm for Tiago interactions."""
    # GUI setup
    window = tk.Tk()
    window.title("Tieni premuto per parlare con Tiago.")
    window.geometry("800x640")
    window.wm_attributes("-topmost", True)    # Set window to always appear on top

    # Create recorder with window as master
    recorder = AudioRecorder(window)

    def on_press(event):
        """Starts the recording when button is pressed."""
        try:
            recorder.start_recording()
            event.widget.config(bg="green", text="🎤 Registrando...\nRilascia per fermare")
            print("[DEBUG] Recording started...")
        except Exception as e:
            print(f"Error starting recording: {e}")
            event.widget.config(bg="red", text="Errore!")

    def on_release(event):
        """Stops the recording and starts the audio processing for an llm response."""
        try:
            audio_bytes = recorder.stop_recording()
            event.widget.config(bg="lightgray", text="Tieni premuto\nper registrare")
            print(f"[DEBUG] Recording stopped. Audio size: {len(audio_bytes)} bytes")
            
            # process the audio_bytes
            transcription = transcribe(audio_bytes, "it")
            print(f"[DEBUG]You said: \n{transcription}")
            llm_answer = ask_llm(transcription)
            print(f"[DEBUG]The LLM responded: \n{llm_answer}")
            say(llm_answer, language="it")
            
            
        except Exception as e:
            print(f"Error stopping recording: {e}")
            event.widget.config(bg="red", text="Errore!")

    # Set up recording button
    recording_button = tk.Button(window, 
                                 text="Tieni premuto\nper registrare", 
                                 font=("liberation sans", 12, "bold"),
                                 width=20,
                                 height=10,
                                 bg="lightgray"
                                 )
    recording_button.pack(expand=True)      # center the button
      
    # Bind mouse press and release
    recording_button.bind("<ButtonPress-1>", on_press)
    recording_button.bind("<ButtonRelease-1>", on_release)

    # Ensure pyaudio properly released when window closes
    def on_window_close():
        try:
            # Stop recording if it's currently active
            if recorder.recording:
                recorder.stop_recording()
            
            # Cancel any pending after calls
            if recorder.after_id:
                window.after_cancel(recorder.after_id)
            
            # Terminaterecorder
            recorder.terminate()
        except Exception as e:
            print(f"Error during cleanup: {e}")
        finally:
            # Force destroy the window
            window.quit()  # This exits the mainloop
            window.destroy()  # This destroys the window
    
    window.protocol("WM_DELETE_WINDOW", on_window_close)
    
    return window, recorder



# Main execution
if __name__ == "__main__":
    interaction_gui, recorder = create_record_window()
    interaction_gui.mainloop()


