from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import time
import threading


# PyQtのウィンドウ表示
app = pg.mkQApp("Plotting Example")
# PyQtのウィンドウ関連
win = pg.GraphicsLayoutWidget(show=True, title="Basic plotting examples")
win.resize(1000, 600)
win.setWindowTitle('pyqtgraph example: Plotting')
pg.setConfigOptions(antialias=True)

# プロット画面の準備
p = win.addPlot(title="real-time line plot")
curve = p.plot(pen='c')  # 引数penで線の色や描画方法を指定

# プロットデータの初期化
fps = 200
n_samples = 1000
fo = 1
data = np.zeros(n_samples)

# グローバル変数
iter = 0
prev_time = time.time()  # 前回の更新時刻

def update():
    global curve, data, iter, fps, fo, n_samples, data_sin, prev_time
    
    fo = 0.1 + iter / n_samples
    t = (1.0 / fps) * iter
    idx = iter % n_samples
    data[idx] = data_sin
    
    pos = idx + 1 if idx < n_samples else 0
    curve.setData(np.r_[data[pos:n_samples], data[0:pos]])
    iter += 1
    
    # 現在の時刻と前回の更新時刻から更新速度を計算
    current_time = time.time()
    update_rate = 1 / (current_time - prev_time)
    prev_time = current_time
    
    print("Update rate:", update_rate)
    
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1 / fps * 1000)

if __name__ == '__main__':
    def get_data():
        global data_sin
        startTime = time.perf_counter()

        while True:
            data_sin = np.sin(2 * np.pi * 0.3 * (time.perf_counter() - startTime))

            time.sleep(0.004)

    data_thread = threading.Thread(target=get_data)
    data_thread.setDaemon(True)
    data_thread.start()

    pg.exec()
