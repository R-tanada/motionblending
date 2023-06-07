# -*- coding: utf-8 -*-
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import time
import threading

class RealTimePlot():
    plot_data = 0
    def __init__(self, framerate = 200, seconds = 5) -> None:
        self.n_samples = int(framerate * seconds)
        self.iter = 0
        RealTimePlot.plot_data = 0
        self.data = np.zeros(self.n_samples)
        self.setWindow(framerate)
        # self.startWindow()

    def setWindow(self, framerate):
        def update():
            print(111)
            idx = self.iter % self.n_samples
            self.data[idx] = self.plot_data
            
            pos = idx + 1 if idx < self.n_samples else 0
            self.curve.setData(np.r_[self.data[pos:self.n_samples], self.data[0:pos]])
            self.iter += 1      

        # PyQtのウィンドウ表示
        app = pg.mkQApp("Plotting Example")
        # PyQtのウィンドウ関連
        win = pg.GraphicsLayoutWidget(show=True, title="Basic plotting examples")
        win.resize(1000, 600)
        win.setWindowTitle('pyqtgraph example: Plotting')
        pg.setConfigOptions(antialias=True)

        # プロット画面の準備
        p = win.addPlot(title="real-time line plot")
        self.curve = p.plot(pen='c')  # 引数penで線の色や描画方法を指定

        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(1 / framerate * 1000)
        pg.exec()

  

    # def startWindow(self):
        

    
class DataManager():
    def __init__(self) -> None:
        self.data_sin = 0
        data_thread = threading.Thread(target=self.get_data)
        data_thread.setDaemon(True)
        data_thread.start()

    def get_data(self):
        startTime = time.perf_counter()
        
        while True:
            self.data_sin = RealTimePlot.plot_data = np.sin(2 * np.pi * 1 * (time.perf_counter() - startTime))
            time.sleep(0.004)


if __name__ == '__main__':
    datamanager = DataManager()
            
    plot = RealTimePlot(datamanager.data_sin)