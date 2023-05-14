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
            position, isMoving = self.posRetained = next(self.predictedPosition), True
        except StopIteration:
            position, isMoving = self.posRetained, False

        return position, isMoving
    
    def GetRotation(self):
        try:
            rotation, isMoving = self.rotRetained = next(self.predictedRotation), True
        except StopIteration:
            rotation, isMoving = self.rotRetained, False

        return rotation, isMoving
    
    def GetGripperValue(self):
        try:
            gripper, isMoving = self.gripRetained = next(self.predictedGripper), True
        except StopIteration:
            gripper, isMoving = self.gripRetained, False

        return gripper, isMoving

    def MonitoringMotion(self, position, rotation, gripper):
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