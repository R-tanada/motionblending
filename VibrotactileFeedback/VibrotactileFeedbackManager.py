import pyaudio
import numpy as np

class VibrotactileFeedbackManager:
    def __init__(self, index) -> None:
        self.sin = np.sin(2.0 * np.pi * np.arange(int(44100/200)) * float(200) / float(44100))
        self.amp = 0
        self.data_out = 0
        p = pyaudio.PyAudio()
        stream = self.OpenStream(p, index)
        stream.start_stream()

    def OpenStream(self, p, index):
        stream = p.open(
            format = pyaudio.paInt16, 
            channels = 1, 
            rate = 44100, 
            output = True,
            frames_per_buffer = int(44100/200), 
            output_device_index = index,
            stream_callback = self.Callback
        )

        return stream
    
    def Callback(self, in_data, frame_count, time_info, status):
        out_data = (self.amp * self.data_out * self.sin).astype(np.int16)
        return (out_data, pyaudio.paContinue)