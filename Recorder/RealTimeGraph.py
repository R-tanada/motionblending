"""
This example demonstrates many of the 2D plotting capabilities
in pyqtgraph. All of the plots may be panned/scaled by dragging with 
the left/right mouse buttons. Right click on any plot to show a context menu.
"""

import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import time
import threading

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
data = []
ptr = 0
def update():
    global curve, data, ptr, p6
    curve.setData(data)
    if ptr == 0:
        p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr += 1
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

startTime = time.perf_counter()

if __name__ == '__main__':
    def startApp():
        pg.exec()
        
    app_thread = threading.Thread(target=startApp)

    try:
        while True:
            data.append(np.sin(2 * np.pi * 5 * time.perf_counter() - startTime))
            print(data)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print('finish')
