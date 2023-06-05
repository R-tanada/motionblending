import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np

# アプリケーションを作成
app = QtGui.QApplication([])

# ウィンドウを作成
win = pg.GraphicsWindow(title="Real-time Plot")
win.resize(800, 600)

# プロットウィジェットを作成
plot = win.addPlot(title="Real-time Data")
curve = plot.plot(pen='y')

# データを保持するリストを作成
data = []

# データ更新のための関数
def update():
    # 最新の100個のデータを表示
    curve.setData(data[-100:])

# タイマーを作成して定期的にupdate関数を呼び出す
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)  # 100ミリ秒ごとに更新

# メインループでデータを受け取り
while True:
    # データを受け取る処理
    new_data = np.random.rand()
    
    # データリストに追加
    data.append(new_data)
    
    # アプリケーションのイベントを処理
    QtGui.QApplication.processEvents()

# アプリケーションを実行
app.exec_()
