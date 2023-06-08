import time

import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import butter, lfilter


class RealTimeLowpassFilter:
    def __init__(self, cutoff_freq, fs, order=5, listNum = 3):
        self.b, self.a = self.butter_lowpass(cutoff_freq, fs, order=order)
        self.z = np.zeros((max(len(self.a), len(self.b))-1, listNum))

        self.init_time = time.perf_counter()
        self.start_time = 10
        self.finish = True

    def butter_lowpass(self, cutoff_freq, fs, order = 1):
        nyquist_freq = 0.5 * fs
        normalized_cutoff_freq = cutoff_freq / nyquist_freq
        b, a = butter(order, normalized_cutoff_freq, btype='low', analog=False)
        return b, a

    def apply(self, data):
        timer = time.perf_counter() - self.init_time
        filtered_data = np.zeros_like(data)  # フィルタリングされたデータの配列を用意
        for i, x in enumerate(data):
            filtered_data[i], self.z[:,i] = lfilter(self.b, self.a, [x], zi=self.z[:,i])

        if timer > self.start_time:
            if self.finish == True:
                print('filtered')
                self.finish = False
            return filtered_data

        else:
            return data
    
if __name__ == '__main__':
    freq = 200
    loopTime = 1/200

    filter = RealTimeLowpassFilter(cutoff_freq=3, fs=200, order=2, listNum=1)
    filtbox = []
    timebox = []
    startTime = time.perf_counter()
    init_data = [np.random.rand()]

    try:
        while True:
            loopstartTime = time.perf_counter()
            data = np.random.rand()

            filtbox.append(filter.apply([data], init_data))
            timebox.append(time.perf_counter() - startTime)
            print(time.perf_counter() - startTime)
            
            time.sleep(loopTime - (time.perf_counter() - loopstartTime))

    except KeyboardInterrupt:
        plt.plot(timebox, filtbox)
        plt.show()
