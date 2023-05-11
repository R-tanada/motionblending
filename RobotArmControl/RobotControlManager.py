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

        self.loopTime = 180

    def SendDataToRobot(self, isEnablexArm: bool = False, isFixedFrameRate = True, isPrintFrameRate = True):
        if isFixedFrameRate:
            windll.winmm.timeBeginPeriod(1)

        # ----- Process info ----- #
        listFrameRate = []
        loopCount = 0
        taskStartTime = 0

        # ----- Control flags ----- #
        isMoving    = False

        try:
            while True:
                if isMoving:
                    loopStartTime = time.perf_counter()

                    if isEnablexArm:
                        self.xarmManager.SendDataToRobot(self.cyberneticManager.GetSharedTransform())
                        self.xarmManager.CheckError()

                    # if loopCount % 20 == 0 and isPrintFrameRate:
                    #     if loopCount != 0:
                    #         listFrameRate.append(1 / (time.perf_counter() - loopStartTime))
                    #         print("Average FPS: ", sum(listFrameRate)/len(listFrameRate))

                    # if isFixedFrameRate:
                    #     self.FixFrameRate(time.perf_counter - loopStartTime)

                    # loopCount += 1
                else:
                    keycode = input('Input > "s": start control \n')

                    if keycode == 's':
                        self.cyberneticManager.SetParticipantInitMotion()

                        isMoving    = True
                        taskStartTime = time.perf_counter()

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

