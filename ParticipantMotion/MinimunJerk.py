import numpy as np
import CustomFunction.CustomFunction as cf

class MinimumJerk:
    def __init__(self, Target: list, xArmConfig: dict, Threshold = 200) -> None:
        initPos = xArmConfig['InitPos']
        initRot = cf.Convert2InverseMatrix(cf.Euler2Quaternion(xArmConfig['InitRot']))
        self.predictedPosition = []
        self.predictedRotation = []
        self.predictedGripper = []
        self.Threshold = Threshold
        self.target = Target
        self.q_init = []
        for target in self.target:
            target['position'] -= np.array(initPos)
            target['rotation'] = np.dot(cf.Convert2Matrix(cf.Euler2Quaternion(target['rotation'])), np.dot(initRot, [0, 0, 0, 1]))
        self.target_index = 0
        self.flag = False

    def GetPosition(self):
        try:
            position = self.posRetained = next(self.predictedPosition)
            isMoving = True
        except StopIteration:
            position, isMoving = self.posRetained, False

        return position, isMoving
    
    def GetRotation(self):
        try:
            rotation = self.rotRetained = np.dot(cf.Convert2Matrix(self.q_init), next(self.predictedRotation))
            isMoving = True
        except StopIteration:
            rotation, isMoving = self.rotRetained, False

        return rotation, isMoving
    
    def GetGripperValue(self):
        try:
            gripper = self.gripRetained = next(self.predictedGripper)
            isMoving = True
        except StopIteration:
            gripper, isMoving = self.gripRetained, False

        return gripper, isMoving

    def MonitoringMotion(self, position, rotation, gripper):
        isMoving = False
        diff = np.linalg.norm(self.target[self.target_index]['position'] - np.array(position))

        if diff > self.Threshold:
            self.flag = True

        if self.flag == True:
            if diff <= self.Threshold:
                self.CreateMotionData(position, rotation, gripper, self.target[self.target_index], 'Liner')
                self.target_index += 1
                if self.target_index == 2:
                    self.target_index = 0
                isMoving = True
                self.flag = False


        return isMoving

    def CreateMotionData(self, position, rotation, gripper, target, motion):
        diffPos = []
        diffRot = []
        flameLength = 500

        def CreateMotion_Liner(target, data, split):
            motionlist = np.linspace(data, target, split)
            return motionlist
        
        def CreateMotion_Liner_Rot(target, data, split):
            weight_list = np.linspace(0, 1, split)
            q_list = []
            for weight in weight_list:
                q_list.append(cf.Slerp_Quaternion(target, data, weight))
            
            return q_list

        def CreateMotion_Sin(target, data, split):
            flamelist = np.linspace(0, np.pi/2, split)
            motionlist = (target - data) * np.sin(flamelist) + data
            return motionlist

        if motion == 'Liner':
            for i in range(3):
                diffPos.append(CreateMotion_Liner(target['position'][i], position[i], flameLength))
            for j in range(4):
                diffRot.append(CreateMotion_Liner_Rot(target['rotation'][j], rotation[0][j], flameLength))
            diffGrip = [850] * flameLength
            diffGrip = np.concatenate([diffGrip, CreateMotion_Liner(target['gripper'], gripper, 500)], 0)

        self.predictedPosition = self.ConvertToIterator(np.transpose(np.array(diffPos)))
        self.predictedRotation = self.ConvertToIterator(np.transpose(np.array(diffRot)))
        self.predictedGripper = self.ConvertToIterator(diffGrip)

    def ConvertToIterator(self, data):
        return iter(data)
