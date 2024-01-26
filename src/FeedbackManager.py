import threading
import time

import numpy as np
import pyaudio
import serial


class Vibrotactile:
    def __init__(self, index) -> None:
        self.index = index
        self.rate = 48000
        self.freq = 150
        self.chunk = int(self.rate / self.freq)
        self.sin = np.sin(2.0 * np.pi * np.arange(self.chunk) * self.freq / self.rate)
        self.amp = 4
        self.data_out = 0

        if self.index != "None":
            self.p = pyaudio.PyAudio()
            self.stream = self.p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.rate,
                output=True,
                frames_per_buffer=self.chunk,
                output_device_index=index,
                stream_callback=self.callback,
            )
            self.stream.start_stream()

    def callback(self, in_data, frame_count, time_info, status):
        # self.data_out = 0
        out_data = (int(self.amp * self.data_out) * self.sin).astype(np.int16)
        # print(int(self.amp * self.data_out))
        return (out_data, pyaudio.paContinue)

    def close(self):
        if self.index != "None":
            self.p.terminate()
        else:
            pass


class LED_Feedback:
    def __init__(self, port) -> None:
        self.data_out = 0
        if port != "None":
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(1)
            send_thread = threading.Thread(target=self.send_data)
            send_thread.setDaemon(True)
            send_thread.start()

    def send_data(self):
        try:
            while True:
                self.ser.write(bytes([self.data_out]))

                time.sleep(0.005)

        except KeyboardInterrupt:
            self.ser.close()
            print("closed socket")


if __name__ == "__main__":
    vibro = Vibrotactile(38)
    vibro2 = Vibrotactile(36)
    vibro.data_out = 255
    vibro2.data_out = 255
    start_time = time.perf_counter()

    try:
        while True:
            print(time.perf_counter() - start_time)
            time.sleep(0.005)

    except KeyboardInterrupt:
        vibro.stream.stop_stream()
        vibro.stream.close()
        vibro.close()
        vibro2.stream.stop_stream()
        vibro2.stream.close()
        vibro2.close()
        print("finish loop")
