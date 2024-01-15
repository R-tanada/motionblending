#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Gripper Control
Please make sure that the gripper is attached to the end.
"""
from __future__ import print_function
import socket
from contextlib import closing
import os
import sys
import time

host = '192.168.1.100'  # 自身のIPアドレス
port =  8888
bufsize = 1024

with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
	sock.settimeout(2)  #timeout
	sock.bind((host,port))
	# sock.listen(1)

	while True:
		try:
			# print(sock.recv(bufsize))   #Python2:only ASCII,Python3:impossible
			data = sock.recv(bufsize).decode()
			result = data
			# addr = sock.accept()
			print(result)   #Python2:Japanese,Python3:anything

		except socket.error as e:
			if str(e) != "timed out":
				print("Error: %s' % e")