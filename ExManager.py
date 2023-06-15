# from ctypes import windll
from re import S

from RobotArmControl.RobotControlManager import RobotControlManager

if __name__ == '__main__':
    robotControlManager = RobotControlManager(isDebug = True)
    robotControlManager.SendDataToRobot(FrameRate = 240, isPrintFrameRate = False)

    print('\n----- End program: ExManager.py -----')
