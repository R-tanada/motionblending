from src.ParticipantMotionManager import ParticipantManager
from src.SensorManager import GripperSensorManager
from src.prediction import MinimumJerk
from src.DataManager import DataRecordManager

class AvatarMotion:
    def __init__(self) -> None:
        self.particpantmotion = ParticipantManager()
        self.predicction = MinimumJerk()

    def set_motion(self, elaspedTime):
        self.particpantmotion.get_participant_motion(elaspedTime)

        self.predicction.MonitoringMotion
