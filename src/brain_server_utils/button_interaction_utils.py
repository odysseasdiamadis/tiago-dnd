import io, wave
from typing import Optional
import pyaudio
import tkinter as tk
from tkinter import font as tkfont
import time


# from brain_server_interaction import BrainInteractor, CONFIG_YAML


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
        print("Recording was started!")

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

        print("Recording was stopped!")

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
    

def create_record_window(on_audio_ready=None):
    """Creates the UI to send a vocal message and interact with an llm for Tiago interactions.
    
        @param on_audio_ready:  Optional callback function that receives audio_bytes when recording stops.
                                If None, no processing is done (just prints debug info).
    """
    # GUI setup
    window = tk.Tk()
    window.title("Tieni premuto per parlare con Tiago.")
    window.geometry("800x640")
    window.wm_attributes("-topmost", True)

    # Create recorder with window as master
    recorder = AudioRecorder(window)
    
    # Flag to avoid multiple simultaneous processes (this stops the user to interact with button until processing is cvompleted)
    is_processing = False

    def on_press(event):
        """Starts the recording when button is pressed."""
        # Block if still processing previous audio
        if is_processing:
            print("[DEBUG] Still processing lasr audio, ignoring press")
            return
            
        try:
            recorder.start_recording()
            event.widget.config(bg="green", text="Sto registrando...\nRilascia per fermare")
            print("[DEBUG] Recording started...")
        except Exception as e:
            print(f"Error starting recording: {e}")
            event.widget.config(bg="red", text="Errore!")

    def on_release(event):
        """Stops the recording and passes audio bytes to callback."""
        nonlocal is_processing
        
        # Don't process if not currently recording
        if not recorder.recording:
            return
            
        try:
            audio_bytes = recorder.stop_recording()
            button = event.widget
            
            # Set processing flag to block new recordings
            is_processing = True
            
            # Show processing state and disable button
            button.config(bg="yellow", text="Elaborazione in corso, pulsante disattivato...", state="disabled")
            window.update()  # Force UI to update
            
            print(f"[DEBUG] Recording stopped. Audio size: {len(audio_bytes)} bytes")
            print("[DEBUG] Processing started - UI blocked")
            
            # Process audio (blocking - UI will freeze until is done)
            try:
                if on_audio_ready:
                    on_audio_ready(audio_bytes)
                print("[DEBUG] Processing complete")
            except Exception as e:
                print(f"Error processing audio: {e}")
                button.config(bg="red", text="Errore!!!")
                window.update()
                window.after(2000, lambda: None)  # Show an error for 2 seconds
            finally:
                # Always reset state when done
                is_processing = False
                button.config(bg="lightgray", text="Tieni premuto\nper registrare", state="normal")
                window.update()
            
        except Exception as e:
            print(f"Error stopping recording: {e}")
            event.widget.config(bg="red", text="Errore!!!", state="normal")
            is_processing = False

    # Set up recording button
    recording_button = tk.Button(window, 
                                 text="Tieni premuto\nper registrare", 
                                 font=("liberation sans", 12, "bold"),
                                 width=20,
                                 height=10,
                                 bg="lightgray"
                                 )
    recording_button.pack(expand=True)
      
    # Bind mouse press and release
    recording_button.bind("<ButtonPress-1>", on_press)
    recording_button.bind("<ButtonRelease-1>", on_release)

    # Ensure pyaudio is properly released when window closes
    def on_window_close():
        try:
            if recorder.recording:
                recorder.stop_recording()
            
            if recorder.after_id:
                window.after_cancel(recorder.after_id)
            
            recorder.terminate()
        except Exception as e:
            print(f"Error during cleanup: {e}")
        finally:
            window.quit()
            window.destroy()
    
    window.protocol("WM_DELETE_WINDOW", on_window_close)
    
    return window, recorder


# Main execution
if __name__ == "__main__":
    
    # TODO: custom handler (customize when using with Tiago)
    def handle_audio(audio_bytes):
        print(f"Got {len(audio_bytes)} bytes of audio!")    
        # Simulate processing time (transcription, LLM, TTS, etc.)
        time.sleep(3) # change, of course    
        print("Process complete!")
    
    interaction_gui, recorder = create_record_window(on_audio_ready=handle_audio)
    interaction_gui.mainloop()