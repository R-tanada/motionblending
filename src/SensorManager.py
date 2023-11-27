import time

import serial


class GripperSensorManager:
    def __init__(self, ComPort, BandRate) -> None:
        self.sensorValue = 850
        self.ComPort = ComPort
        print(self.ComPort)

        if ComPort != "None":
            self.serialObject = serial.Serial(ComPort, BandRate)
            # self.serialObject.close()

        else:
            pass

    def StartReceiving(self):
        try:
            while True:
                if self.ComPort != "None":
                    data = self.serialObject.readline()
                    self.sensorValue = float(data.strip().decode("utf-8"))
                    # print(self.sensorValue)

                else:
                    self.sensorValue = 850

                time.sleep(0.005)

        except KeyboardInterrupt:
            print("KeyboardInterrupt >> Stop: BendingSensorManager.py")


class FootSwitchManager:
    def __init__(self) -> None:
        self.flag = False
        print("hello")

    def detect_sensor(self):
        time.sleep(3)

        try:
            while True:
                key = input("press to start predicition")
                if key == "f":
                    self.flag = True
                    print("----- foot switch pressed -----")

                time.sleep(0.005)

        except:
            print("error occured")
