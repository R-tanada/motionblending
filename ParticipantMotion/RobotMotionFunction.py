import numpy as np

class RobotMotionFunction:
    def __init__(self, TargetPosAndRot, completionTime, TargetGrip, gripTime, Mount, AutoGrip) -> None:
        initPosAndRot = np.array([370, 0, 320, 180, 0, 90])
        self.AutoGrip = AutoGrip
        self.frequency = 150
        self.motionThreshold = 200
        self.completionTime = completionTime
        self.count = 0
        self.motion_flag = 0
        self.ChangeInitFlag = False
        self.MotionFeedbackFlag = False
        self.RobotMotionFlag = 'Arm_1'
        self.return_pos = self.return_rot = self.return_grip = []
        self.TargetPosAndRot = TargetPosAndRot - initPosAndRot
        self.TargetPosAndRot[0], self.TargetPosAndRot[1], self.TargetPosAndRot[2], self.TargetPosAndRot[3], self.TargetPosAndRot[4], self.TargetPosAndRot[5] = -1 * self.TargetPosAndRot[2], self.TargetPosAndRot[1], self.TargetPosAndRot[0], -1 * self.TargetPosAndRot[5], self.TargetPosAndRot[3], -1 * self.TargetPosAndRot[4]
        # print(TargetPosAndRot)
        self.CreateDefaultMovement(TargetPosAndRot, initPosAndRot, TargetGrip, gripTime, completionTime, Mount)

    def CreateDefaultMovement(self, target, init, targetgrip, griptime, time, mount):
        diffList = [0] * 6

        def CreateMotionList(target, init, split):
            motionlist = np.linspace(init, target, split)
            motionlist = motionlist - init
            return motionlist

        self.splitNum_motion = self.frequency * time
        self.splitNum_grip = self.frequency * griptime

        for i in range(6):
            diffList[i] = CreateMotionList(target[i], init[i], self.splitNum_motion)

        self.RobotPos, self.RobotRot = diffList[0:3], diffList[3:6]
        self.RobotPos, self.RobotRot = np.array(self.RobotPos) , np.array(self.RobotRot)
        self.RobotPos, self.RobotRot = np.transpose(self.RobotPos) , np.transpose(self.RobotRot)

        if mount == 'right':
            self.RobotPos[:, 2], self.RobotRot[: ,1] , self.RobotRot[:, 2] = -1 * self.RobotPos[:, 2], -1 * self.RobotRot[:, 1], -1 * self.RobotRot[:, 2]
            self.RobotPos, self.RobotRot = self.RobotPos[:, [2, 1, 0]], self.RobotRot[:, [2, 0, 1]]
        elif mount == 'front':
            self.RobotPos, self.RobotRot = self.RobotPos[:, [1, 2, 0]], self.RobotRot[:, [1, 2, 0]]

        self.GripperPos = np.linspace(850, targetgrip, self.splitNum_grip)



    def RecordedMotion(self):
        try:
            return_pos, return_rot = self.RobotPos[self.count], self.RobotRot[self.count]
            self.count += 1
            return return_pos, return_rot
        except StopIteration:
            pass

    def ChangeToRobotMotion(self, position, rotation, grip):
        TargetDistance = np.linalg.norm(position - self.TargetPosAndRot[0:3])
        # print(TargetDistance)

        if self.motion_flag == 0:
            if TargetDistance <= self.motionThreshold:
                self.motion_flag = 1
                self.CreateMotionList_v2(position, rotation)

        if self.motion_flag:
            self.MotionFeedbackFlag = True

            if self.RobotMotionFlag == 'Arm_1':
                self.return_pos, self.return_rot, self.return_grip = self.RobotPos[self.count], self.RobotRot[self.count], 850
                self.count += 1
                if self.count == self.splitNum_motion - 1:
                    self.RobotMotionFlag = 'Gripper_1'
                    self.count = 0

            elif self.RobotMotionFlag == 'Gripper_1':
                if self.AutoGrip:
                    self.return_grip = self.GripperPos[self.count]
                    self.count += 1
                    if self.count == self.splitNum_grip - 1:
                        self.RobotMotionFlag = 'User_2'
                        self.count = 0
                        self.DiffRobotAndUserPos, self.DiffRobotAndUserRot = self.return_pos - position, self.return_rot - rotation

                else:
                    self.return_grip = grip

            elif self.RobotMotionFlag == 'User_2':
                self.return_pos, self.return_rot, self.return_grip = position + self.DiffRobotAndUserPos, rotation + self.DiffRobotAndUserRot, grip

        else:
            self.return_pos, self.return_rot, self.return_grip = position, rotation, grip

        return self.return_pos, self.return_rot, self.return_grip

    def ChangeToRobotMotionGrad(self, position, rotation, grip):
        TargetDistance = np.linalg.norm(position - self.TargetPosAndRot[0:3])
        # print(TargetDistance)

        if self.motion_flag == 0:
            if TargetDistance <= self.motionThreshold:
                self.motion_flag = 1
                self.CreateMotionList_v2(position, rotation)

        if self.motion_flag:
            self.MotionFeedbackFlag = True

            userWeight = (TargetDistance/self.motionThreshold)
            if userWeight <= 0:
                userWeight = 0
            elif userWeight >= 1:
                userWeight = 1

            if self.RobotMotionFlag == 'Arm_1':
                self.return_pos, self.return_rot, self.return_grip = userWeight * position + (1 - userWeight) * self.RobotPos[self.count], userWeight * position + (1 - userWeight) * self.RobotRot[self.count], 850
                self.count += 1
                if self.count == self.splitNum_motion - 1:
                    self.RobotMotionFlag = 'Gripper_1'
                    self.count = 0

            elif self.RobotMotionFlag == 'Gripper_1':
                if self.AutoGrip:
                    self.return_grip = self.GripperPos[self.count]
                    self.count += 1
                    if self.count == self.splitNum_grip - 1:
                        self.RobotMotionFlag = 'User_2'
                        self.count = 0
                        self.DiffRobotAndUserPos, self.DiffRobotAndUserRot = self.return_pos - position, self.return_rot - rotation

                else:
                    self.return_grip = grip

            elif self.RobotMotionFlag == 'User_2':
                self.return_pos, self.return_rot, self.return_grip = position + self.DiffRobotAndUserPos, rotation + self.DiffRobotAndUserRot, grip

        else:
            self.return_pos, self.return_rot, self.return_grip = position, rotation, grip

        return self.return_pos, self.return_rot, self.return_grip

    def CreateMotionList_v2(self, CurrentPosition, CurrentRotation):
        diffList = [0] * 6
        CurrentMotion = np.append(CurrentPosition, CurrentRotation)

        def CreateMotionList(target, current, split):
            motionlist = np.linspace(current, target, split)
            return motionlist

        self.splitNum_motion = self.frequency * self.completionTime

        print(diffList, self.TargetPosAndRot, CurrentMotion)

        for i in range(6):
            diffList[i] = CreateMotionList(self.TargetPosAndRot[i], CurrentMotion[i], self.splitNum_motion)

        self.RobotPos, self.RobotRot = diffList[0:3], diffList[3:6]
        self.RobotPos, self.RobotRot = np.array(self.RobotPos) , np.array(self.RobotRot)
        self.RobotPos, self.RobotRot = np.transpose(self.RobotPos) , np.transpose(self.RobotRot)
        print(self.RobotPos)
        print(self.RobotRot)

    def CreateMotionList(self, CurrentPosition, CurrentRotation):
        StartMotionIndex = np.abs(self.RobotPos[:, 0] - CurrentPosition[0]).argsort()[0].tolist()
        print('index:{}'.format(StartMotionIndex))
        self.RobotPos = self.RobotPos[StartMotionIndex:]
        self.RobotRot = self.RobotRot[StartMotionIndex:]

        self.RobotPos[:, 0] = self.CalculateMotion(self.RobotPos[:, 0], CurrentPosition[0])
        self.RobotPos[:, 1] = self.CalculateMotion(self.RobotPos[:, 1], CurrentPosition[1])
        self.RobotPos[:, 2] = self.CalculateMotion(self.RobotPos[:, 2], CurrentPosition[2])
        self.RobotRot[:, 0] = self.CalculateMotion(self.RobotRot[:, 0], CurrentRotation[0])
        self.RobotRot[:, 1] = self.CalculateMotion(self.RobotRot[:, 1], CurrentRotation[1])
        self.RobotRot[:, 2] = self.CalculateMotion(self.RobotRot[:, 2], CurrentRotation[2])

    def CalculateMotion(self, dataList, startValue):
        def CreateLinerList(initdiff, dataList):
            listLenght = len(dataList)
            motionList = np.arange(listLenght)
            return -(initdiff/listLenght) * motionList + initdiff

        initdiff = startValue - dataList[0]
        diffList = CreateLinerList(initdiff, dataList)

        return (dataList + diffList)

if __name__ == "__main__":
    # target = [392.8, -172.8, 31.2, 179.6, 41.9, 90]
    target = [370, 0, 320, 200, 0, 90]
    targetgrip = 300
    robotmotion = RobotMotionFunction(target, 4, targetgrip, 2, Mount = 'right', AutoGrip = True)
