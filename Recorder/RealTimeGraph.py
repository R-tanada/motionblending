"""
This example demonstrates many of the 2D plotting capabilities
in pyqtgraph. All of the plots may be panned/scaled by dragging with 
the left/right mouse buttons. Right click on any plot to show a context menu.
"""

import numpy as np
import threading
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

class RealTimeGraph():
    def __init__(self) -> None:
        self.ptr = 0
        self.data = []
        app_thread = threading.Thread(target=self.SetWindow)
        # app_thread.setDaemon(True)
        app_thread.start()
        # self.SetWindow()

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
        data = np.random.normal(size=(10,1000))
        self.ptr = 0
        def update():
            curve.setData(data[self.ptr%10])
            if self.ptr == 0:
                p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
            self.ptr += 1
        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(50)

        pg.exec()

if __name__ == '__main__':
    graph = RealTimeGraph()
    print('hello')