import numpy as np
from scipy.signal import butter, lfilter
from matplotlib import pyplot as plt
import time

# ローパスフィルタの設計
def butter_lowpass(cutoff_freq, fs, order = 5):
    nyquist_freq = 0.5 * fs
    normalized_cutoff_freq = cutoff_freq / nyquist_freq
    b, a = butter(order, normalized_cutoff_freq, btype='low', analog=False)
    return b, a

# リアルタイムでデータにローパスフィルタを適用するクラス
class RealTimeLowpassFilter:
    def __init__(self, cutoff_freq, fs, order=5):
        self.b, self.a = butter_lowpass(cutoff_freq, fs, order=order)
        self.z = np.zeros((max(len(self.a), len(self.b))-1, len(self.b)))

    def apply(self, data):
        filtered_data = np.zeros_like(data)
        for i in range(len(data)):
            filtered_data[i], self.z[:, i] = lfilter(self.b, self.a, data[i], zi=self.z[:, i])
        return filtered_data

# サンプリング周波数とカットオフ周波数の設定
fs = 200  # サンプリング周波数 [Hz]
cutoff_freq = 10  # カットオフ周波数 [Hz]

# リアルタイムローパスフィルタのインスタンス化
rt_filter = RealTimeLowpassFilter(cutoff_freq, fs)
before_data = []
filterd_data = []
time_list = []
start_time = time.perf_counter()
before_time = 0
freq = 200
target_time = 1/freq

# データのリアルタイム処理
try:
    while True:
        st = time.perf_counter()
        t = time.perf_counter() - start_time
        
        # データの取得（仮想的な例としてランダムデータを生成）
        time_list.append(t)
        data = np.random.randn(1)  # サンプルデータの取得
        data = [data, data]
        before_data.append(data)
        # ローパスフィルタの適用
        filterd_data.append(rt_filter.apply(data))
        # print(data)

        if target_time > (time.perf_counter() - st):
            time.sleep(target_time - (time.perf_counter() - st))

        freq_time = t - before_time
        before_time = t
        print(freq_time)

except KeyboardInterrupt:

    plt.plot(time_list, before_data)
    plt.plot(time_list, filterd_data)
    plt.show()
