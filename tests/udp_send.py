# -*- coding: utf-8 -*-
from __future__ import print_function

import socket
import time
from contextlib import closing

# host = 'localhost'

# host = '192.168.1.100'  # 送り先のIPアドレス
host = "127.0.0.1"
port = 8888
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    while True:
        data_right = "right," + str(100)
        data_left = "left," + str(300)
        message_right = data_right.encode("utf-8")
        message_left = data_left.encode("utf-8")
        print(message_left, message_right)
        sock.sendto(message_right, (host, port))
        sock.sendto(message_left, (host, port))
        time.sleep(1)
