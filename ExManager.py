from ctypes import windll
from re import S
import json
import time

from RobotArmControl.RobotControlManager import RobotControlManager
from CyberneticAvatarMotion.CyberneticAvatarMotionManager import \
    CyberneticAvatarMotionManager

if __name__ == '__main__':
    with open('SettingFile/settings_dual_fusion.json', 'r') as settings_file:
        settings = json.load(settings_file)

    xArmConfigs = settings['xArmConfigs']
    ParticipantConfigs = settings['ParticipantsConfigs']

    cyberneticManager = CyberneticAvatarMotionManager(ParticipantConfigs, xArmConfigs)
    robotControlManager = RobotControlManager(xArmConfigs)

    windll.winmm.timeBeginPeriod(1)

    isEnablexArm = True
    global loopTime, flameList
    loopTime = 1/ 240

    try:
        while True:
            if isMoving:
                loopStartTime = time.perf_counter()

                if isEnablexArm:
                    robotControlManager.SendDataToRobot(cyberneticManager.GetSharedTransform())

                FixFrameRate(time.perf_counter() - loopStartTime)
                if isPrintFrameRate:
                    CheckFrameRate(time.perf_counter() - loopStartTime)

            else:
                keycode = input('Input > "s": start control \n')

                if keycode == 's':
                    cyberneticManager.SetParticipantInitMotion()
                    isMoving    = True

    except:
        print('----- Exception has occurred -----')
        import traceback
        traceback.print_exc()

        windll.winmm.timeEndPeriod(1)

    except KeyboardInterrupt:
        print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')

        # ----- Disconnect ----- #
        if isEnablexArm:
            xArmManager.DisConnect()
            print('successfully disconnected')

        windll.winmm.timeEndPeriod(1)

            

    print('\n----- End program: ExManager.py -----')

def FixFrameRate(processDuration):
    sleepTime = self.loopTime - processDuration
    if sleepTime < 0:
        pass
    else:
        time.sleep(sleepTime)

def CheckFrameRate(loopTime):
    self.FrameList.append(1/ loopTime)
    if len(self.FrameList) == 30:
        print(sum(self.FrameList)/ len(self.FrameList))
        self.FrameList = []