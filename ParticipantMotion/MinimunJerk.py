import numpy as np

class MinimumJerk:
    def __init__(self, TargetPosition: list, Threshold) -> None:
        self.InitPos = {}
        self.predictedPosition = []
        self.predictedRotation = []
        self.predictedGripper = []
        self.dataList = []
        self.targetPosition = TargetPosition - self.InitPos
        self.Threshold = Threshold
        self.dt = 1/ 240

    def GetPosition(self):
        try:
            position = self.posRetained = next(self.predictedPosition)
        except StopIteration:
            position = self.posRetained

        return position
    
    def GetRotation(self):
        try:
            rotation = self.rotRetained = next(self.predictedRotation)
        except StopIteration:
            rotation = self.rotRetained

        return rotation
    
    def GetGripperValue(self):
        try:
            gripper = self.gripRetained = next(self.predictedGripper)
        except StopIteration:
            gripper = self.gripRetained

        return gripper

    def MonitoringMotion(self, position):
        isMoving = False
        velocity, acceleration = self.CalculateMotionInfo(position)

        if np.linalg.norm(self.targetPosition - position):
            self.CreateMinimumJerkMotion(position, velocity, acceleration)
            isMoving = True

        return isMoving

    def CreateMinimumJerkMotion(self, position, velocity, acceleration):
        pass

    def CalculateMotionInfo(self, data):
        velocity = acceleration = 0
        self.dataList.append(data)

        if len(self.dataList) == 3:
            velocity = self.CalculateVelocity(self.dataList)
            acceleration = self.CalculateAcceleration(self.dataList)
            del self.dataList[0]

        return velocity, acceleration

    def CalculateVelocity(self, dataList):
        return np.diff(dataList, n=1, axis=0)[2] / self.dt
    
    def CalculateAcceleration(self, dataList):
        return np.diff(dataList, n=2, axis=0)[1] / self.dt**2
    
    def ConvertToIteration(self, data):
        return iter(data)