import numpy as np
from scipy.signal import butter, filtfilt
import time
from matplotlib import pyplot as plt

# ローパスフィルタの設計
def butter_lowpass(cutoff_freq, fs, order=1):
    nyquist_freq = 0.5 * fs
    normalized_cutoff_freq = cutoff_freq / nyquist_freq
    b, a = butter(order, normalized_cutoff_freq, btype='low', analog=False)
    return b, a

# サンプリング周波数とカットオフ周波数の設定
fs = 200  # サンプリング周波数 [Hz]
cutoff_freq = 3  # カットオフ周波数 [Hz]

# フィルタ係数の計算
b, a = butter_lowpass(cutoff_freq, fs)

before_data = []
filterd_data = []
time_list = []
start_time = time.perf_counter()
before_time = 0
freq = 200
target_time = 1/freq
data_before = 0

# リアルタイムでデータにフィルタを適用する
try:
    while True:
        st = time.perf_counter()
        t = time.perf_counter() - start_time
        # データの取得（仮想的な例としてランダムデータを生成）
        time_list.append(t)
        data = np.random.randn(1)  # サンプルデータの取得
        before_data.append(data)

        # リアルタイムでフィルタを適用
        filtered_data = filtfilt(b, a, [data, data_before])
        filterd_data.append(filtered_data[0])
        print(filtered_data[0])
        data_before = data

        # フィルタリングされたデータの利用
        # ここに必要な処理を追加してください
        if target_time > (time.perf_counter() - st):
            time.sleep(target_time - (time.perf_counter() - st))

        freq_time = t - before_time
        before_time = t
        print(freq_time)
except KeyboardInterrupt:

    plt.plot(time_list, before_data)
    plt.plot(time_list, filterd_data)
    plt.show()
