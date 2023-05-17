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
    isMoving = False

    try:
        while True:
            if isMoving:
                loopStartTime = time.perf_counter()

                if isEnablexArm:
                    robotControlManager.SendDataToRobot(cyberneticManager.GetSharedTransform())

            else:
                keycode = input('Input > "s": start control \n')

                if keycode == 's':
                    cyberneticManager.SetParticipantInitMotion()
                    isMoving    = True

    except KeyboardInterrupt:
        print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')
        if isEnablexArm:
            robotControlManager.DisConnect()
            print('successfully disconnected')

        windll.winmm.timeEndPeriod(1)

    except:
        print('----- Exception has occurred -----')
        import traceback
        traceback.print_exc()

        windll.winmm.timeEndPeriod(1)



            

    print('\n----- End program: ExManager.py -----')