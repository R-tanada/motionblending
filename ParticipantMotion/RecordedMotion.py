import csv
import sys

import numpy as np
from MotionFilter.MotionFilter import MotionFilter


class RecordedMotion:
    def __init__(self, MotionPath, GripPath, threshold) -> None:
        self.motionFilter = MotionFilter()
        self.motionThreshold = threshold
        self.motion_flag = 0
        self.count = 0
        recordedPos, recordedRot, recordedGrip = self.InputData(MotionPath, GripPath)
        recordedPos, recordedRot = self.CreateNoiseDataSet(recordedPos, recordedRot, frequency = 0.5, scale = 40)
        print(recordedPos, recordedRot, recordedGrip)
        self.recordedPos_Iter, self.recordedRot_Iter, self.recordedGrip_Iter = self.ConvertToIterator(recordedPos), self.ConvertToIterator(recordedRot), self.ConvertToIterator(recordedGrip)

    def InputData(self, motionpath, grippath):
        with open(motionpath) as f:
            reader = csv.reader(f)
            motion_data = [row for row in reader][1:]
            motion_data = [[float(v) for v in row] for row in motion_data]
            motion_data = np.array(motion_data)

        recordedPos = motion_data[:, 1:4] * 0.5
        recordedRot = motion_data[:, 4:7] * 0.5

        with open(grippath) as f:
            reader = csv.reader(f)
            motion_data = [row for row in reader][1:]
            motion_data = [[float(v) for v in row] for row in motion_data]

        recordedGrip = np.array(motion_data) * 0.5

        return recordedPos, recordedRot, recordedGrip

    def CreateNoiseDataSet(self, position, rotation, frequency, scale):
        noisePos = []
        noiseRot = []
        noisePos_filt = []
        noiseRot_filt = []

        for i in range(3):
            noisePos.append(np.random.normal(loc = 0, scale = scale, size = len(position)))
            noiseRot.append(np.random.normal(loc = 0, scale = scale, size = len(rotation)))

        self.motionFilter.InitLowPassFilterWithOrder(samplerate = 180, fp = frequency, n = 2)

        for j in range(3):
            noisePos_filt.append(self.motionFilter.ButterFilter(noisePos[j]))
            noiseRot_filt.append(self.motionFilter.ButterFilter(noiseRot[j]))

        noisePos_filt = np.transpose(np.array(noisePos_filt))
        noiseRot_filt = np.transpose(np.array(noiseRot_filt))

        return (position + noisePos_filt), (rotation + noiseRot_filt)

    def ConvertToIterator(self, data):
        return iter(data)

    def FusionWithRecordedMotion(self, position, rotation, gripper):
        try:
            robotPos = np.array(position) * 0.5 + next(self.recordedPos_Iter)
            robotRot = np.array(rotation) * 0.5 + next(self.recordedRot_Iter)
            robotGrip = gripper * 0.5 + next(self.recordedGrip_Iter)
        except StopIteration:
            sys.exit()

        return robotPos, robotRot, robotGrip

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
