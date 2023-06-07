"""
This example demonstrates many of the 2D plotting capabilities
in pyqtgraph. All of the plots may be panned/scaled by dragging with 
the left/right mouse buttons. Right click on any plot to show a context menu.
"""

import numpy as np
import threading
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import time

class RealTimeGraph():
    def __init__(self) -> None:
        self.ptr = 0
        self.data = []
        self.SetWindow()

    def SetWindow(self):
        app = pg.mkQApp("Plotting Example")
        #mw = QtWidgets.QMainWindow()
        #mw.resize(800,800)

        win = pg.GraphicsLayoutWidget(show=True, title="Basic plotting examples")
        win.resize(1000,600)
        win.setWindowTitle('pyqtgraph example: Plotting')

        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        p6 = win.addPlot(title="Updating plot")
        curve = p6.plot(pen='y')
        def update():
            self.data = get_data()
            curve.setData(self.data)

        def get_data():
            global data
            print(len(data))
            return data
        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(30)

        pg.exec()

if __name__ == '__main__':
    data = []
    def data_input():
        startTime = time.perf_counter()

        try:
            while True:
                data.append(np.sin(2 * np.pi * 50 * (time.perf_counter() - startTime)))
                if len(data) == 100:
                    del data[0]

                time.sleep(0.005)

        except KeyboardInterrupt:
            print('finish')

    data_thread = threading.Thread(target=data_input)
    data_thread.setDaemon(True)
    data_thread.start()

    graph = RealTimeGraph()