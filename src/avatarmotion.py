from src.ParticipantMotionManager import ParticipantManager
from src.SensorManager import GripperSensorManager
from src.MinimunJerk import MinimumJerk
from src.DataManager import DataRecordManager

class AvatarMotion:
    def __init__(self) -> None:
        self.particpantMotionManager = ParticipantManager()
        self.minimumjerk = MinimumJerk()

    def set_motion(self, elaspedTime):
        
        self.particpantMotionManager.
