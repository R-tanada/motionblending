import time

import numpy as np
from matplotlib import pyplot as plt


class RobotMotion:
    def __init__(self, TargetPosition, TargetRotation, MaxFlame, Threshold, AutoGrip, TargetGrip = 500, GripFlame = 500, Motion = 'Liner') -> None:
        self.initPos = np.array([280, 0, 250])
        self.initRot = np.array([180, 0, 0])
        self.StartToTarget = np.linalg.norm(TargetPosition - self.initPos)
        self.targetPos = np.array(TargetPosition) - self.initPos
        self.targetRot = np.array(TargetRotation) - self.initRot
        self.threshold = Threshold
        self.maxFlame = MaxFlame
        self.AutoGrip = AutoGrip
        self.VibrotactileFeedback = False
        self.RobotMotion = 'before'
        self.posList = []
        self.motion = Motion
        self.flamelist = 0
        if self.AutoGrip == True:
            gripPos = self.CreateGripMotion(TargetGrip, GripFlame)
            self.gripPos_Iter = self.ConvertToIterator(gripPos)

    def ChangeToRobotMotion(self, position, rotation, gripper, loopcount, tn):
        diffToTarget = np.linalg.norm(position - self.targetPos)
        print(diffToTarget)

        if self.RobotMotion == 'before':
            return_pos, return_rot, return_grip = position, rotation, gripper

            if diffToTarget <= self.threshold:
                self.loopCount = loopcount
                self.tn = tn
                robotPos, robotRot = self.CreateRobotMotion(position, rotation, diffToTarget)
                self.robotPos_Iter, self.robotRot_Iter = self.ConvertToIterator(robotPos), self.ConvertToIterator(robotRot)
                self.RobotMotion = 'enable'
                self.VibrotactileFeedback = True

        if self.RobotMotion == 'enable':
            try:
                return_pos, return_rot, return_grip = next(self.robotPos_Iter), next(self.robotRot_Iter), gripper
            except StopIteration:
                self.diffPos_UserToTarget = self.targetPos - position
                self.diffRot_UserToTarget = self.targetRot - rotation
                self.VibrotactileFeedback = False
                if self.AutoGrip == True:
                    self.RobotMotion = 'grip'
                else:
                    self.RobotMotion = 'after'

        if self.RobotMotion == 'grip':
                try:
                    return_pos, return_rot, return_grip = position + self.diffPos_UserToTarget, rotation + self.diffRot_UserToTarget, next(self.gripPos_Iter)
                except StopIteration:
                    self.RobotMotion = 'after'

        if self.RobotMotion == 'after':
            return_pos, return_rot, return_grip = position + self.diffPos_UserToTarget, rotation + self.diffRot_UserToTarget, 500

        return return_pos, return_rot, return_grip

    def CreateRobotMotion(self, CurrentPosition, CurrentRotation, diffToTarget):
        diffPos = []
        diffRot = []

        flameLength = int(self.maxFlame * (diffToTarget / self.StartToTarget))

        def CreateMotion_Liner(targetPos, Pos, split):
            motionlist = np.linspace(Pos, targetPos, split)
            return motionlist

        def CreateMotion_Sin(targetPos, Pos, split):
            self.flamelist = np.linspace(0, np.pi/2, split)
            motionlist = (targetPos - Pos) * np.sin(self.flamelist) + Pos
            return motionlist

        def CreateMotion_Sigmoid(targetPos, Pos, split):
            self.flamelist = np.linspace(-4, 4, split)
            motionlist = (targetPos - Pos) * (1 / (1 + np.exp(-self.flamelist))) + Pos
            return motionlist

        def CreateMotion_MinimunJerk(targetPos, Pos, initPos = 0, t_target = 5):
            A_x = np.array([[t_target**3, t_target**4, t_target**5, (t_target-self.tn)/120],
                            [3*t_target**2, 4*t_target**3, 5*t_target**4, 1/120],
                            [6*t_target, 12*t_target**2, 20*t_target**3, 0],
                            [self.tn**3, self.tn**4, self.tn**5, 0]])
            b_x = np.array([targetPos-initPos, 0, 0, Pos-initPos])
            X_x = np.linalg.solve(A_x, b_x)
            self.flamelist = np.linspace(self.tn, t_target, int((self.loopCount/self.tn)*(t_target-self.tn)))
            dataset = initPos + X_x[0]*(self.flamelist**3) + X_x[1]*(self.flamelist**4) + X_x[2]*(self.flamelist**5) + X_x[3]*((self.flamelist-self.tn)/120)
            return dataset

        for i in range(3):
            if self.motion == 'Liner':
                diffPos.append(CreateMotion_Liner(self.targetPos[i], CurrentPosition[i], flameLength))
                diffRot.append(CreateMotion_Liner(self.targetRot[i], CurrentRotation[i], flameLength))

            elif self.motion == 'Sin':
                diffPos.append(CreateMotion_Sin(self.targetPos[i], CurrentPosition[i], flameLength))
                diffRot.append(CreateMotion_Sin(self.targetRot[i], CurrentRotation[i], flameLength))

            elif self.motion == 'Sigmoid':
                diffPos.append(CreateMotion_Sigmoid(self.targetPos[i], CurrentPosition[i], flameLength))
                diffRot.append(CreateMotion_Sigmoid(self.targetRot[i], CurrentRotation[i], flameLength))

            elif self.motion == 'MinimumJerk':
                diffPos.append(CreateMotion_MinimunJerk(self.targetPos[i], CurrentPosition[i]))
                diffRot.append(CreateMotion_MinimunJerk(self.targetRot[i], CurrentRotation[i]))

        robotPos, robotRot = np.transpose(np.array(diffPos)) , np.transpose(np.array(diffRot))

        return robotPos, robotRot

    def CreateGripMotion(self, TargetValue, Flame):

        def CreateMotion_Liner(targetPos, Pos, split):
            motionlist = np.linspace(Pos, targetPos, split)
            return motionlist

        gripPos = CreateMotion_Liner(TargetValue, 850, Flame)

        return np.transpose(np.array(gripPos))

    def ConvertToIterator(self, ListConverted):
        return iter(ListConverted)

if __name__ == '__main__':
    targetPos = [474.2, 1.5, 120]
    targetRot = [-178.1, -24.5, 26.5]
    targetGrip = 500
    currentPos = [500, -100, 150]
    currentRot = [100, 5, 90]
    currentGrip = 850
    loopcount = 600
    tn = 3.5

    robotMotion = RobotMotion(targetPos, targetRot, MaxFlame = 200, Threshold = 400, AutoGrip = True, Motion = 'Sin')
    # robotMotion.CreateRobotMotion(Pos, currentRot, 100)

    while True:
        position, rotation, gripper= robotMotion.ChangeToRobotMotion(currentPos - robotMotion.initPos, currentRot, currentGrip, loopcount, tn)
        print(position, rotation, gripper)
        time.sleep(0.01)
