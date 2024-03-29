import time

import serial


class GripperSensorManager:
    def __init__(self, ComPort, BandRate) -> None:
        self.sensorValue   = 850
        self.ComPort = ComPort

        if ComPort != 'None':
            self.serialObject = serial.Serial(ComPort, BandRate)

        else:
            pass

    def StartReceiving(self):
        try:
            while True:
                if self.ComPort != 'None':
                    data = self.serialObject.readline()
                    self.sensorValue = float(data.strip().decode('utf-8'))

                else:
                    self.sensorValue = 850

                time.sleep(0.005)

        except KeyboardInterrupt:
            print('KeyboardInterrupt >> Stop: BendingSensorManager.py')

