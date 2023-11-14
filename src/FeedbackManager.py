import time

import numpy as np
import pyaudio


class Vibrotactile:
    def __init__(self) -> None:
        self.rate = 48000
        self.freq = 200
        self.chunk = int(self.rate / self.freq)
        self.sin = np.sin(2.0 * np.pi * np.arange(self.chunk) * self.freq / self.rate)
        self.amp = 11000
        self.data_out = 0
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.rate,
            output=True,
            frames_per_buffer=self.chunk,
            output_device_index=19,
            stream_callback=self.callback,
        )
        self.stream.start_stream()

    def callback(self, in_data, frame_count, time_info, status):
        out_data = (int(self.amp * self.data_out) * self.sin).astype(np.int16)
        # print(int(self.amp * self.data_out))
        return (out_data, pyaudio.paContinue)

    def close(self):
        self.p.terminate()


if __name__ == "__main__":
    vibro = Vibrotactile()
    vibro.data_out = 100
    start_time = time.perf_counter()

    try:
        while True:
            print(time.perf_counter() - start_time)
            time.sleep(0.005)

    except KeyboardInterrupt:
        vibro.stream.stop_stream()
        vibro.stream.close()
        vibro.close()
        print("finish loop")
