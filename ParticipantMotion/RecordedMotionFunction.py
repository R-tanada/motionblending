import csv
from difflib import diff_bytes
import numpy as np

class RecordedMotionFunction:
    def __init__(self, MotionPath, GripPath, threshold) -> None:
        self.motionThreshold = threshold
        self.motion_flag = 0
        self.count = 0
        self.InputData(MotionPath, GripPath)
        # self.CreateMotionIterator

    def InputData(self, motionpath, grippath):
        with open(motionpath) as f:
            reader = csv.reader(f)
            motion_data = [row for row in reader][1:]
            motion_data = [[float(v) for v in row] for row in motion_data]
            motion_data = np.array(motion_data)

        self.recordedPos = motion_data[:7000, 1:4]
        self.recordedRot = motion_data[:7000, 4:7]
        print(self.recordedPos)
        print(self.recordedRot)

        with open(grippath) as f:
            reader = csv.reader(f)
            motion_data = [row for row in reader][1:]
            motion_data = [[float(v) for v in row] for row in motion_data]

        self.recordedGrip = np.array(motion_data)
        self.recordedGrip = self.recordedGrip[:7000]
        print(self.recordedGrip)

    def CreateMotionIterator(self):
        self.iter_pos = iter(self.recordedPos)
        self.iter_rot = iter(self.recordedRot)
        self.iter_grip = iter(self.recordedGrip)

    def RecordedMotion(self, loopcount):
        try:
            return_pos, return_rot, return_grip = self.recordedPos[loopcount], self.recordedRot[loopcount], self.recordedGrip[loopcount]
            return return_pos, return_rot, return_grip
        except StopIteration:
            pass

    def ChangeToRecordedMotion(self, position, rotation, grippos):
        # print(position[0])
        if self.motion_flag == 0:
            if position[0] >= self.motionThreshold:
                self.motion_flag = 1
                self.CreateMotionList(position, rotation)

        if self.motion_flag:
            try:
                return_pos, return_rot, return_grip = self.recordedPos[self.count], self.recordedRot[self.count], self.recordedGrip[self.count]
                print(return_pos, return_rot)
                self.count += 1
                return return_pos, return_rot, return_grip
            except StopIteration:
                self.motion_flag = 0
                return position, rotation, grippos

        else:
            return position, rotation, grippos

    def CreateMotionList(self, CurrentPosition, CurrentRotation):
        StartMotionIndex = np.abs(self.recordedPos[1:, 0] - CurrentPosition[0]).argsort()[0].tolist()
        print('index:{}'.format(StartMotionIndex))
        self.recordedPos = self.recordedPos[StartMotionIndex:]
        self.recordedRot = self.recordedRot[StartMotionIndex:]
        self.recordedGrip = self.recordedGrip[StartMotionIndex:]

        self.recordedPos[:, 1] = self.CalculateMotion(self.recordedPos[:, 1], CurrentPosition[1])
        self.recordedPos[:, 2] = self.CalculateMotion(self.recordedPos[:, 2], CurrentPosition[2])
        self.recordedRot[:, 0] = self.CalculateMotion(self.recordedRot[:, 0], CurrentRotation[0])
        self.recordedRot[:, 1] = self.CalculateMotion(self.recordedRot[:, 1], CurrentRotation[1])
        self.recordedRot[:, 2] = self.CalculateMotion(self.recordedRot[:, 2], CurrentRotation[2])

    def CalculateMotion(self, dataList, startValue):
        def CreateLinerList(initdiff, dataList):
            listLenght = len(dataList)
            motionList = np.arange(listLenght)
            return -(initdiff/listLenght) * motionList + initdiff

        initdiff = startValue - dataList[0]
        print(startValue)
        print(initdiff)
        diffList = CreateLinerList(initdiff, dataList)
        print(diffList)
        print(dataList)
        print(dataList + diffList)

        return (dataList + diffList)

if __name__ == '__main__':
    current_pos = [8.5, 1.5,-100]
    current_rot = [-0.08, 8.5,-1]
    recMotion = RecordedMotionFunction('/Users/yuzu/Documents/GitHub/ms-proj-sharedavatar/Experiment1/Python/Recorder/RecordedData/___Transform_Participant_1_20221208_2258.csv', '/Users/yuzu/Documents/GitHub/ms-proj-sharedavatar/Experiment1/Python/Recorder/RecordedData/___GripperValue_1_20221208_2258.csv', threshold = 200)
    pos, rot = recMotion.CreateMotionList(current_pos, current_rot)
