from src.participantmotion import ParticipantManager
from src.MinimunJerk import MinimumJerk
from src.RobotControlManager import RobotControlManager
from src.DataManager import DataLoadManager, DataPlotManager, DataRecordManager

class MotionCntrol:
    def __init__(self, is_simulation, is_visualize) -> None:


        self.participantmotion = ParticipantManager()
        self.motionblending = MinimumJerk()

    def set_motion(self):
        motion_data = self.participantmotion.GetParticipantMotion()

        if self.motionblending.MonitoringMotion():
            motion_data = self.motionblending.GetPosition()

        self.robotcontrol.SendDataToRobot(motion_data)

    def set_init_motion(self):
        pass