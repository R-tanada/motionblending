# -----------------------------------------------------------------------
# Author:   Takayoshi Hagiwara (KMD)
# Created:  2021/8/25
# Summary:  Experiment manager
# -----------------------------------------------------------------------

from ctypes import windll
from re import S

import RobotArmController.RobotControlManager

if __name__ == '__main__':
    robotControlManager = RobotArmController.RobotControlManager.RobotControlManagerClass()
    robotControlManager.SendDataToRobot(participantNum=1, executionTime=9999, isFixedFrameRate=False, frameRate=200, isChangeOSTimer=True, isExportData=False, isEnablexArm=True)

    print('\n----- End program: ExManager.py -----')
