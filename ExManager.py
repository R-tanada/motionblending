from re import S

from RobotArmControl.RobotControlManager import RobotControlManager

if __name__ == '__main__':
    robotControlManager = RobotControlManager(is_Debug = True, is_Recording = False, is_Plotting = True)
    # robotControlManager.SendDataToRobot(FrameRate = 240, isPrintFrameRate = True)
    robotControlManager.SendDataToRobot(FrameRate = 240, isPrintFrameRate = False)

    print('\n----- End program: ExManager.py -----')
