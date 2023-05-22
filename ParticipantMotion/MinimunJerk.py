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
            if target['rotation'][3] < 0:
                target['rotation'] = -target['rotation']
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

    def CreateMotionMinimumJerk(self, tn, loopCount, pos_n, vel_n, acc_n, tf, pos_f, vel_f = [0, 0, 0], acc_f = [0, 0, 0]):
        a_matrix = [
            [1, tn, tn**2,  tn**3,      tn**4,      tn**5       ],
            [0, 1,  2*tn,   3*(tn**2),  4*(tn**3),  5*(tn**4)   ],
            [0, 0,  2,      6*tn,       12*(tn**2), 20*(tn**3)  ],
            [1, tf, tf**2,  tf**3,      tf**4,      tf**5       ],
            [0, 1,  2*tf,   3*(tf**2),  4*(tf**3),  5*(tf**4)   ],
            [0, 0,  2,      6*tf,       12*(tf**2), 20*(tf**3)  ]
        ]
        b_matrix = [pos_n, vel_n, acc_n, pos_f, vel_f, acc_f]
        coeff = np.linalg.solve(a_matrix, b_matrix)

        def function(coeff, x):
            return coeff[0] + coeff[1]*x + coeff[2]*(x**2) + coeff[3]*(x**3) + coeff[4]*(x**4) + coeff[5]*(x**5)
        
        flameLength = (loopCount/ tn) * (tf - tn)
        flame = np.linspace(tn, tf, int(flameLength))

        position = []
        for i in range(3):
            position.append(function(coeff[:, i], flame))

        self.predictedPosition = iter(np.transpose(position))