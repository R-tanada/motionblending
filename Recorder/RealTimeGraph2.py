import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QTimer

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # ウィンドウの設定
        self.setWindowTitle("Real-time Graph")
        self.setGeometry(100, 100, 800, 600)

        # グラフウィジェットの作成
        self.graphWidget = pg.PlotWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.graphWidget)

        # レイアウトとウィジェットの設定
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        # グラフの設定
        self.graphWidget.setBackground("w")
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setLabel("left", "Value")
        self.graphWidget.setLabel("bottom", "Time")

        # グラフのカーブを作成
        self.curve = self.graphWidget.plot()

        # タイマーの作成とスロットの設定
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)  # 30fps（約33ミリ秒ごと）に更新

    def update_plot(self):
        # データの取得
        x_data, y_data = self.get_data()

        # グラフの描画
        self.curve.setData(x_data, y_data)

    def get_data(self):
        # データの取得処理を実装
        # ここでは仮のデータを生成して返す
        x_data = np.arange(0, 10, 0.1)
        y_data = np.sin(x_data)
        return x_data, y_data

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
