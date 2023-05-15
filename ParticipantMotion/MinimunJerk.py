import numpy as np

class MinimumJerk:
    def __init__(self, Target: list, Threshold = 200) -> None:
        initPos = [200, 0, 450]
        initRot = [180, 0, 0]
        self.predictedPosition = []
        self.predictedRotation = []
        self.predictedGripper = []
        self.dataList = []
        self.Threshold = Threshold
        self.target = Target
        for target in self.target:
            target['position'] -= initPos
            target['rotation'] -= initRot
        self.dt = 1/ 240
        self.target_index = 0
        self.flag = False

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
        diff = np.linalg.norm(self.target[self.target_index]['position'] - position)
        # velocity, acceleration = self.CalculateMotionInfo(position)

        if diff > self.Threshold:
            self.flag = True

        if self.flag == True:
            if diff <= self.Threshold:
                self.CreateMotionData(position, rotation, gripper, self.target[self.target_index], 'Sin')
                self.target_index += 1
                if self.target_index == 3:
                    self.target_index = 0
                isMoving = True
                self.flag = False


        return isMoving

    # def CreateMinimumJerkMotion(self, position, velocity, acceleration):
    #     pass

    # def CalculateMotionInfo(self, data):
    #     velocity = acceleration = 0
    #     self.dataList.append(data)

    #     if len(self.dataList) == 3:
    #         velocity = self.CalculateVelocity(self.dataList)
    #         acceleration = self.CalculateAcceleration(self.dataList)
    #         del self.dataList[0]

    #     return velocity, acceleration

    # def CalculateVelocity(self, dataList):
    #     return np.diff(dataList, n=1, axis=0)[2] / self.dt
    
    # def CalculateAcceleration(self, dataList):
    #     return np.diff(dataList, n=2, axis=0)[1] / self.dt**2

    def CreateMotionData(self, position, rotation, gripper, target, motion):
        diffPos = []
        diffRot = []
        flameLength = 6000

        def CreateMotion_Liner(target, data, split):
            motionlist = np.linspace(data, target, split)
            return motionlist

        def CreateMotion_Sin(target, data, split):
            flamelist = np.linspace(0, np.pi/2, split)
            motionlist = (target - data) * np.sin(flamelist) + data
            return motionlist

        if motion == 'Liner':
            for i in range(3):
                diffPos.append(CreateMotion_Liner(target['position'][i], position[i], flameLength))
                diffRot.append(CreateMotion_Liner(target['rotation'][i], rotation[i], flameLength))
                diffGrip = [850] * flameLength
                diffGrip.append(CreateMotion_Liner(target['gripper'], gripper, 1000))

        elif motion == 'Sin':
            for i in range(3):
                diffPos.append(CreateMotion_Sin(target['position'][i], position[i], flameLength))
                diffRot.append(CreateMotion_Sin(target['rotation'][i], rotation[i], flameLength))
                diffGrip = [850] * flameLength
                diffGrip.append(CreateMotion_Sin(target['gripper'], gripper, 1000))

        self.predictedPosition = self.ConvertToIterator(np.transpose(np.array(diffPos)))
        self.predictedRotation = self.ConvertToIterator(np.transpose(np.array(diffRot)))
        self.predictedGripper = self.ConvertToIterator(diffGrip)

    def ConvertToIterator(self, data):
        return iter(data)