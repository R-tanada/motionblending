import serial
from UDP.UDPManager import UDPManager


class GripperSensorManager:
    def __init__(self, ip, port) -> None:
        self.ip             = ip
        self.port           = port
        self.sensorValue   = 850

        self.serialObject = serial.Serial(ip, port)

    def StartReceiving(self):
        try:
            while True:
                data = self.serialObject.readline()
                self.sensorValue = float(data.strip().decode('utf-8'))

        except KeyboardInterrupt:
            print('KeyboardInterrupt >> Stop: BendingSensorManager.py')

