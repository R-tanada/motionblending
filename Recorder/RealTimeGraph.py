import sys
import random
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PySide6.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt6agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # ウィンドウの設定
        self.setWindowTitle("Real-time Graph")
        self.setGeometry(100, 100, 800, 600)

        # レイアウトとウィジェットの作成
        layout = QVBoxLayout()
        self.widget = QWidget()
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

        # Matplotlibのフィギュアとキャンバスの作成
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        # グラフデータの初期化
        self.x_data = []
        self.y_data = []

        # タイマーの作成とスロットの設定
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(1000)  # 1秒ごとに更新

    def update_plot(self):
        # データの更新
        self.x_data.append(len(self.x_data) + 1)
        self.y_data.append(random.randint(0, 100))

        # グラフの描画
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        ax.plot(self.x_data, self.y_data)

        # グラフを再描画
        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
