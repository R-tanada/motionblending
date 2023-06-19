import platform
# from ctypes import windll
import json
import time

from CyberneticAvatarMotion.CyberneticAvatarMotionManager import CyberneticAvatarMotionManager

class ExManager:
    def __init__(self, is_Simulation: bool = True, is_Visualize: bool = True) -> None:
        self.frameList = []

        with open('SettingFile/settings_single.json', 'r') as settings_file:
            settings = json.load(settings_file)

        cyberneticManager = CyberneticAvatarMotionManager(settings['ParticipantsConfigs'], settings['xArmConfigs'])

        if is_Simulation == True:
            if is_Visualize == True:
                from RobotArmControl.SimulationManager import SimulationManager
                robotManager = SimulationManager(settings['xArmConfigs'])
            else:
                pass

        else:
            from RobotArmControl.RobotControlManager import RobotControlManager
            robotManager = RobotControlManager(settings['xArmConfigs'])

        self.mainLoop(cyberneticManager, robotManager)
            

    def mainLoop(self, cyberneticManager, robotManager):
        isMoving = False

        try:
            while True:
                if isMoving:
                    loopStartTime = time.perf_counter()
                    elapsedTime = time.perf_counter() - initTime

                    robotManager.SendDataToRobot(cyberneticManager.GetSharedTransform())

                    self.FixFrameRate(time.perf_counter() - loopStartTime, 1/50)
                    self.CheckFrameRate(time.perf_counter() - loopStartTime)

                else:
                    self.MonitorKeyEvent(True, robotManager)

                    if keycode == 's':
                        cyberneticManager.SetParticipantInitMotion()
                        initTime = time.perf_counter()
                        isMoving = True

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')
            robotManager.DisConnect()
            print('Successfully Disconnected')

            # windll.winmm.timeEndPeriod(1)

        except:
            print('----- Exception has occurred -----')
            import traceback
            traceback.print_exc()

            # windll.winmm.timeEndPeriod(1)

    def FixFrameRate(self, processDuration, loopTime):
        sleepTime = loopTime - processDuration
        if sleepTime < 0:
            pass
        else:
            time.sleep(sleepTime)

    def CheckFrameRate(self, loopTime):
        self.frameList.append(1/ loopTime)
        if len(self.frameList) == 30:
            print(sum(self.frameList)/ len(self.frameList))
            self.frameList = []

    def MonitorKeyEvent(self, is_Simulation, robotManager):
        if is_Simulation == True:
            robotManager.MonitorKeyEvent()

        else:
            keycode = input('Input > "s": start control \n')

        return 's'
    
if __name__ == '__main__':
    exManager = ExManager()

    print('\n----- End program: ExManager.py -----')
