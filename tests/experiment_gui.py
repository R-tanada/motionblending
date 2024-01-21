import sys
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtNetwork import QUdpSocket
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
        self.left_label = QLabel('-', alignment=Qt.AlignCenter)
        left_layout.addWidget(self.left_label)
        main_layout.addWidget(self.left_widget)

        # 中央の縦線
        line = QWidget()
        line.setFixedWidth(3)  # 太さを変更
        line.setStyleSheet("background-color: black;")
        main_layout.addWidget(line)

        # 右側のウィジェット
        self.right_widget = QWidget(self)
        right_layout = QVBoxLayout(self.right_widget)
        self.right_label = QLabel('-', alignment=Qt.AlignCenter)
        right_layout.addWidget(self.right_label)
        main_layout.addWidget(self.right_widget)

        # UDP通信用の設定
        self.udp_socket = QUdpSocket(self)
        self.udp_socket.bind(8888)  # ポート番号を8888に変更

        # UDPデータ受信時の処理を設定
        self.udp_socket.readyRead.connect(self.process_udp_datagrams)

        # フォントサイズを大きくする
        font = QFont()
        font.setPointSize(300)  # フォントサイズを300に設定
        self.left_label.setFont(font)
        self.right_label.setFont(font)

    def process_udp_datagrams(self):
        while self.udp_socket.hasPendingDatagrams():
            datagram, host, port = self.udp_socket.readDatagram(self.udp_socket.pendingDatagramSize())
            data_list = datagram.data().decode().split(',')
            received_mount = data_list[0]
            received_number = int(data_list[1])
            if received_mount == 'right':
                self.right_label.setText(str(received_number))
            elif received_mount == 'left':
                self.left_label.setText(str(received_number))
            

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SplitWindowApp()
    window.show()
    sys.exit(app.exec_())