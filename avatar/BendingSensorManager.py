# -----------------------------------------------------------------------
# Author:   Takumi Katagiri (Nagoya Institute of Technology), Takayoshi Hagiwara (KMD)
# Created:  2021
# Summary:  曲げセンサからのデータ取得用マネージャー
# -----------------------------------------------------------------------

import serial

class BendingSensorManager:
    
    bendingValue = 0

    def __init__(self, ip, port) -> None:
        self.ip             = ip
        self.port           = port
        self.bufsize        = 4096
        self.bendingValue   = 425

        self.serialObject = serial.Serial(ip, port)
        not_used = self.serialObject.readline()

    def StartReceiving(self):
        try:
            while True:
                data = self.serialObject.readline()
                self.bendingValue = float(data.strip().decode('utf-8'))

        except KeyboardInterrupt:
            print('KeyboardInterrupt >> Stop: BendingSensorManager.py')

