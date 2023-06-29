import platform
import json
import time

class ExManager:
    def __init__(self, is_Simulation: bool = False, is_Visualize: bool = False) -> None:
        self.frameList = []

        with open('docs/settings_single.json', 'r') as settings_file:
            settings = json.load(settings_file)

        from src.CyberneticAvatarMotionManager import CyberneticAvatarMotionManager
        cyberneticManager = CyberneticAvatarMotionManager(settings['ParticipantsConfigs'], settings['xArmConfigs'])

        if is_Simulation == True:
            if is_Visualize == True:
                from src.SimulationManager import SimulationManager
                robotManager = SimulationManager(settings['xArmConfigs'])
            else:
                robotManager = None

        else:
            if is_Visualize == True:
                from src.SimulationManager import SimulationManager
                robotManager = SimulationManager(settings['xArmConfigs'])
            else:
                from src.RobotControlManager import RobotControlManager
                robotManager = RobotControlManager(settings['xArmConfigs'])
            
        time.sleep(0.5)

        self.mainLoop(cyberneticManager, robotManager)
            

    def mainLoop(self, cyberneticManager, robotManager):
        if platform.system() == 'Windows':
            from ctypes import windll
            windll.winmm.timeBeginPeriod(1)

        isMoving = False

        try:
            while True:
                if isMoving:
                    loopStartTime = time.perf_counter()
                    elapsedTime = time.perf_counter() - initTime

                    cyberneticManager.SetElaspedTime(elapsedTime)
                    transform = cyberneticManager.GetSharedTransform()

                    if robotManager != None:
                        robotManager.SendDataToRobot(transform)

                    self.FixFrameRate(time.perf_counter() - loopStartTime, 1/200)
                    # self.CheckFrameRate(time.perf_counter() - loopStartTime)

                else:
                    keycode = self.MonitorKeyEvent(is_Visualize = False, robotManager=robotManager)

                    if keycode == 's':
                        cyberneticManager.SetParticipantInitMotion()
                        initTime = time.perf_counter()
                        isMoving = True

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: mainLoop()')
            if robotManager != None:
                robotManager.DisConnect()
                print('Successfully Disconnected')

            cyberneticManager.PlotGraph()

        except:
            print('----- Exception has occurred -----')
            import traceback
            traceback.print_exc()

        if platform.system() == 'Windows':
            windll.winmm.timeEndPeriod(1)

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

    def MonitorKeyEvent(self, is_Visualize, robotManager):
        if is_Visualize == True:
            keycode = robotManager.MonitorKeyEvent()

        else:
            keycode = input('Input > "s": start control \n')

        return keycode
    
if __name__ == '__main__':
    ExManager(is_Simulation = False, is_Visualize = False)

    print('\n----- End program: ExManager.py -----')