import pyaudio

class VibrotactileFeedbackManager:
    def __init__(self) -> None:
        p = pyaudio()
        stream = self.OpenStream(p)
        stream.start_stream()

    def OpenStream(self, p):
        stream = p.open(
            format = 1, 
            channnels = 1, 
            rate = 1, 
            output = True,
            frames_per_buffer = 1, 
            output_device_index = 1,
            stream_callback = self.callback
        )

        return stream
        

