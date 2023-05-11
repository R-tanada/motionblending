from ctypes import windll
from re import S

from RobotArmControl.RobotControlManager import RobotControlManager

if __name__ == '__main__':
    robotControlManager = RobotControlManager()
    robotControlManager.SendDataToRobot(isEnablexArm = True, isFixedFrameRate = True, isPrintFrameRate = True)

    print('\n----- End program: ExManager.py -----')
