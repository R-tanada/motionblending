import time

import numpy as np
import pyaudio


class Vibrotactile:
    def __init__(self) -> None:
        self.rate = 44100
        self.freq = 200
        self.chunk = int(self.rate / self.freq)
        self.sin = np.sin(2.0 * np.pi * np.arange(self.chunk) * self.freq / self.rate)
        self.amp = 100
        self.data_out = 0
        p = pyaudio.PyAudio()
        stream = self.open_stream(p)
        stream.start_stream()

    def open_stream(self, p):
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.rate,
            output=True,
            frames_per_buffer=self.chunk,
            output_device_index=1,
            stream_callback=self.callback,
        )

        return stream

    def callback(self, in_data, frame_count, time_info, status):
        out_data = (self.amp * self.data_out * self.sin).astype(np.int16)
        print(self.amp * self.data_out * self.sin)
        return (out_data, pyaudio.paContinue)


if __name__ == "__main__":
    vibro = Vibrotactile()
    vibro.data_out = 100
    start_time = time.perf_counter()

    try:
        while True:
            print(time.perf_counter() - start_time)
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("finish loop")
