import sys
from PySide6.QtCore import Qt, QTimer
from PySide6.QtNetwork import QUdpSocket  # 修正: QUdpSocketのimportを追加
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel

class SplitWindowApp(QWidget):
    def __init__(self):
        super().__init__()

        # ウィンドウの初期設定
        self.setWindowTitle('UDP Communication App')
        self.setGeometry(100, 100, 600, 400)

        # レイアウトの設定
        main_layout = QHBoxLayout(self)

        # 左側のウィジェット
        self.left_widget = QWidget(self)
        left_layout = QVBoxLayout(self.left_widget)
        self.left_label = QLabel('0', alignment=Qt.AlignCenter)
        left_layout.addWidget(self.left_label)
        main_layout.addWidget(self.left_widget)

        # 中央の縦線
        line = QWidget()
        line.setFixedWidth(1)
        line.setStyleSheet("background-color: black;")
        main_layout.addWidget(line)

        # 右側のウィジェット
        self.right_widget = QWidget(self)
        right_layout = QVBoxLayout(self.right_widget)
        self.right_label = QLabel('0', alignment=Qt.AlignCenter)
        right_layout.addWidget(self.right_label)
        main_layout.addWidget(self.right_widget)

        # UDP通信用の設定
        self.udp_socket = QUdpSocket(self)
        self.udp_socket.bind(8888)  # ポート番号を適切なものに変更

        # UDPデータ受信時の処理を設定
        self.udp_socket.readyRead.connect(self.process_udp_datagrams)

    def process_udp_datagrams(self):
        while self.udp_socket.hasPendingDatagrams():
            datagram, host, port = self.udp_socket.readDatagram(self.udp_socket.pendingDatagramSize())
            received_number = int(datagram.data().decode())
            self.left_label.setText(str(received_number))
            self.right_label.setText(str(2 * received_number))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SplitWindowApp()
    window.show()
    sys.exit(app.exec_())
