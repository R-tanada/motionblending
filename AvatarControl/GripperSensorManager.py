import serial

class GripperSensorManager:
    
    bendingValue = 0

    def __init__(self, port, bandrate) -> None:
        self.port           = port
        self.bandrate       = bandrate
        self.bufsize        = 4096
        self.bendingValue   = 425

        self.serialObject = serial.Serial(port, bandrate)
        not_used = self.serialObject.readline()

    def StartReceiving(self):
        try:
            while True:
                data = self.serialObject.readline()
                self.sensorValue = float(data.strip().decode('utf-8'))

        except KeyboardInterrupt:
            print('KeyboardInterrupt >> Stop: BendingSensorManager.py')

