import json
import platform
import time

from src.mode_select import mode as mode_decided


class ExManager:
    def __init__(
        self,
        is_Simulation: bool = False,
        is_Visualize: bool = False,
        is_Recording: bool = False,
        is_Plotting: bool = False,
    ) -> None:
        self.frameList = []
        self.is_Visualize = is_Visualize
        self.is_Plotting = is_Plotting
        self.is_Recording = is_Recording

        with open("docs/settings_single.json", "r") as settings_file:
            settings = json.load(settings_file)

        from src.CyberneticAvatarMotionManager import CyberneticAvatarMotionManager

        self.cyberneticManager = CyberneticAvatarMotionManager(
            settings["ParticipantsConfigs"],
            settings["xArmConfigs"],
            is_Simulation,
            is_Recording,
        )

        if is_Simulation == True:
            if is_Visualize == True:
                from src.SimulationManager import SimulationManager

                self.robotManager = SimulationManager(settings["xArmConfigs"])
            else:
                self.robotManager = None

        else:
            if is_Visualize == True:
                from src.SimulationManager import SimulationManager

                self.robotManager = SimulationManager(settings["xArmConfigs"])
            else:
                from src.RobotControlManager import RobotControlManager

                self.robotManager = RobotControlManager(settings["xArmConfigs"])

        time.sleep(0.5)

        self.mainLoop()

    def mainLoop(self, FrameRate: int = 240):
        if platform.system() == "Windows":
            from ctypes import windll

            windll.winmm.timeBeginPeriod(1)

        isMoving = False

        try:
            while True:
                if isMoving:
                    loopStartTime = time.perf_counter()
                    elapsedTime = time.perf_counter() - initTime

                    self.cyberneticManager.SetElaspedTime(elapsedTime)
                    transform = self.cyberneticManager.GetSharedTransform()

                    if self.robotManager != None:
                        self.robotManager.SendDataToRobot(transform)

                    self.FixFrameRate(
                        time.perf_counter() - loopStartTime, 1 / FrameRate
                    )
                    self.CheckFrameRate(time.perf_counter() - loopStartTime)

                else:
                    keycode = self.MonitorKeyEvent(is_Visualize=self.is_Visualize)

                    if keycode == "s":
                        self.cyberneticManager.SetParticipantInitMotion()
                        initTime = time.perf_counter()
                        isMoving = True

        except KeyboardInterrupt:
            print("\nKeyboardInterrupt >> Stop: mainLoop()")
            if self.robotManager != None:
                self.robotManager.DisConnect()
                print("Successfully Disconnected")

            if self.is_Plotting:
                self.cyberneticManager.PlotGraph()

            if self.is_Recording:
                self.cyberneticManager.ExportCSV()

        except:
            print("----- Exception has occurred -----")
            import traceback

            traceback.print_exc()

        if platform.system() == "Windows":
            windll.winmm.timeEndPeriod(1)

    def FixFrameRate(self, processDuration, loopTime):
        sleepTime = loopTime - processDuration
        if sleepTime < 0:
            pass
        else:
            time.sleep(sleepTime)

    def CheckFrameRate(self, loopTime):
        self.frameList.append(1 / loopTime)
        if len(self.frameList) == 30:
            print(sum(self.frameList) / len(self.frameList))
            self.frameList = []

    def MonitorKeyEvent(self, is_Visualize):
        if is_Visualize == True:
            keycode = self.robotManager.MonitorKeyEvent()

        else:
            keycode = input('Input > "s": start control \n')

        return keycode


if __name__ == "__main__":
    is_Recording = False
    mode = mode_decided
    print(mode)
    if mode == 0:
        is_Recording = True
    ExManager(
        is_Simulation=False,
        is_Visualize=False,
        is_Recording=is_Recording,
        is_Plotting=False,
    )

    print("\n----- End program: ExManager.py -----")
