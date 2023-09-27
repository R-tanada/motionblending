import json
import platform
import time
from datamanage import DataRecordManager
from participantmotion import ParticipantManager
from automation import RecordedMotion


class CyberneticAvatarMotionManager:
    def __init__(self, is_Simulation: bool = False, is_Visualize: bool = False, is_Recording: bool = False, is_Plotting: bool = False) -> None:
        self.frameList = []
        self.is_Visualize = is_Visualize
        self.is_Plotting = is_Plotting
        self.is_Recording = is_Recording
        self.is_Simulation = is_Simulation

        with open('docs/settings_single.json', 'r') as settings_file:
            settings = json.load(settings_file)

        self.participant = ParticipantManager(settings['ParticipantConfigs'], settings['xArmConfigs'])
        self.automation = RecordedMotion()
        self.recorder_participant = DataRecordManager()
        self.recorder_time = DataRecordManager()

        if is_Simulation == True:
            if is_Visualize == True:
                from src.bulletcontrol import SimulationManager
                self.robotManager = SimulationManager(settings['xArmConfigs'])
            else:
                self.robotManager = None

        else:
            if is_Visualize == True:
                from src.bulletcontrol import SimulationManager
                self.robotManager = SimulationManager(settings['xArmConfigs'])
            else:
                from src.robotcontrol import RobotControlManager
                self.robotManager = RobotControlManager(settings['xArmConfigs'])

        time.sleep(0.5)

        self.mainLoop()

    def mainLoop(self, FrameRate: int = 240):
        if platform.system() == 'Windows':
            from ctypes import windll
            windll.winmm.timeBeginPeriod(1)

        isMoving = False
        initTime = time.perf_counter()

        avatar_motions = {}

        try:
            while True:
                if isMoving:
                    loopStartTime = time.perf_counter()
                    elapsedTime = time.perf_counter() - initTime

                    participant_motions = self.participant.get_motions()

                    for mount in participant_motions.keys():
                        avatar_motions[mount] = self.automation.MonitoringMotion()
                    

                    if self.robotManager != None:
                        self.robotManager.SendDataToRobot(transform)

                    if self.is_Recording == True and self.is_Simulation == False:
                        self.recorder_time.record(elapsedTime)
                        self.recorder_participant.record(participant_motions)

                    self.fix_frame_rate(time.perf_counter() - loopStartTime, 1/FrameRate)
                    # self.CheckFrameRate(time.perf_counter() - loopStartTime)

                else:
                    keycode = self.monitor_key_event(is_Visualize = self.is_Visualize)

                    if keycode == 's':
                        self.participant.set_init_motions()
                        initTime = time.perf_counter()
                        isMoving = True

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: mainLoop()')
            if self.robotManager != None:
                self.robotManager.DisConnect()
                print('Successfully Disconnected')

        except:
            print('----- Exception has occurred -----')
            import traceback
            traceback.print_exc()

        if platform.system() == 'Windows':
            windll.winmm.timeEndPeriod(1)

    def fix_frame_rate(self, processDuration, loopTime):
        sleepTime = loopTime - processDuration
        if sleepTime < 0:
            pass
        else:
            time.sleep(sleepTime)

    def check_frame_rate(self, loopTime):
        self.frameList.append(1/ loopTime)
        if len(self.frameList) == 30:
            print(sum(self.frameList)/ len(self.frameList))
            self.frameList = []

    def monitor_key_event(self, is_Visualize):
        if is_Visualize == True:
            keycode = self.robotManager.MonitorKeyEvent()

        else:
            keycode = input('Input > "s": start control \n')

        return keycode

if __name__ == '__main__':
    ExManager(is_Simulation = False, is_Visualize = False, is_Recording = False, is_Plotting = True)

    print('\n----- End program: ExManager.py -----')