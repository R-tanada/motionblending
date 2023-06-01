import numpy as np
from scipy.signal import butter, lfilter
from matplotlib import pyplot as plt
import time

class RealTimeLowpassFilter:
    def __init__(self, cutoff_freq, fs, order):
        self.b, self.a = self.butter_lowpass(cutoff_freq, fs, order=order)
        self.z = np.zeros((max(len(self.a), len(self.b))-1, len(self.b)))

    def butter_lowpass(self, cutoff_freq, fs, order = 5):
        nyquist_freq = 0.5 * fs
        normalized_cutoff_freq = cutoff_freq / nyquist_freq
        b, a = butter(order, normalized_cutoff_freq, btype='low', analog=False)
        return b, a

    def apply(self, data):
        filtered_data = np.zeros_like(data)
        for i in range(len(data)):
            filtered_data[i], self.z[:, i] = lfilter(self.b, self.a, data[i], zi=self.z[:, i])
        return filtered_data