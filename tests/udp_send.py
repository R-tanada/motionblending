# -*- coding: utf-8 -*-
from __future__ import print_function
import socket
import time
from contextlib import closing

# host = 'localhost'

host = '192.168.1.130'  # 送り先のIPアドレス
port = 8888
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    while True:
        message = 'hello'.encode('utf-8')
        print(message)
        sock.sendto(message, (host, port))
        time.sleep(1)

	    