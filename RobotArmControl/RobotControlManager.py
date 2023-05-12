import json
import time
from ctypes import windll

import numpy as np
from matplotlib.pyplot import flag

from CyberneticAvatarMotion.CyberneticAvatarMotionManager import \
    CyberneticAvatarMotionManager
# ----- Custom class ----- #
from RobotArmControl.xArmManager import xArmManager


class RobotControlManager:
    def __init__(self) -> None:
        with open('SettingFile/settings_dual_fusion.json', 'r') as settings_file:
            settings = json.load(settings_file)

        xArmConfigs = settings['xArmConfigs']
        ParticipantConfigs = settings['ParticipantsConfigs']

        self.cyberneticManager = CyberneticAvatarMotionManager(ParticipantConfigs, xArmConfigs)
        self.xarmManager = xArmManager(xArmConfigs)

        self.loopTime = 0
        self.FrameList = []

    def SendDataToRobot(self, isEnablexArm: bool = False, FrameRate = 240, isPrintFrameRate = True):
        windll.winmm.timeBeginPeriod(1)
        self.loopTime = 1/ FrameRate

        # ----- Control flags ----- #
        isMoving    = False

        try:
            while True:
                if isMoving:
                    loopStartTime = time.perf_counter()

                    if isEnablexArm:
                        self.xarmManager.SendDataToRobot(self.cyberneticManager.GetSharedTransform())
                        self.xarmManager.CheckError()

                    self.FixFrameRate(time.perf_counter() - loopStartTime)
                    if isPrintFrameRate:
                        self.CheckFrameRate(time.perf_counter() - loopStartTime)

                else:
                    keycode = input('Input > "s": start control \n')

                    if keycode == 's':
                        self.cyberneticManager.SetParticipantInitMotion()
                        isMoving    = True

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')

            # ----- Disconnect ----- #
            if isEnablexArm:
                xArmManager.DisConnect()
                print('successfully disconnected')

            windll.winmm.timeEndPeriod(1)

        except:
            print('----- Exception has occurred -----')
            import traceback
            traceback.print_exc()

            windll.winmm.timeEndPeriod(1)

    def FixFrameRate(self, processDuration):
        sleepTime = self.loopTime - processDuration
        if sleepTime < 0:
            pass
        else:
            time.sleep(sleepTime)

    def CheckFrameRate(self, loopTime):
        self.FrameList.append(1/ loopTime)
        if len(self.FrameList) == 30:
            print(sum(self.FrameList)/ len(self.FrameList))
            self.FrameList = []