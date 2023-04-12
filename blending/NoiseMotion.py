import itertools
import random

import numpy as np
from matplotlib import pyplot as plt
from MotionFilter import MotionFilter


class NoiseMotion:
    def __init__(self, Frequency, Scale, UserControlWeight, Latency = 0) -> None:
        self.posList = []
        self.rotList = []
        self.latency = Latency
        self.controlWeigt = {'user':UserControlWeight, 'robot':1 - UserControlWeight}
        self.motionFilter = MotionFilter()
        noiseSet_Pos, noiseSet_Rot = self.CreateNoiseDataSet(Frequency, Scale)
        self.noiseSet_Pos_Iter, self.noiseSet_Rot_Iter = self.ConvertToIteratorCycle(noiseSet_Pos), self.ConvertToIteratorCycle(noiseSet_Rot)


    def MotionFunction(self, position, rotation, current_time):
        pos_user = np.array(position) * self.controlWeigt['user']
        pos_robot = np.array(position) * self.controlWeigt['robot'] + next(self.noiseSet_Pos_Iter)
        rot_user = np.array(rotation) * self.controlWeigt['user']
        rot_robot = np.array(rotation) * self.controlWeigt['robot'] + next(self.noiseSet_Rot_Iter)

        self.posList.append(pos_robot)
        self.rotList.append(rot_robot)

        if current_time < self.latency:
            return_pos = pos_user
            return_rot = rot_user
        else:
            return_pos = pos_user + self.posList[0]
            return_rot = rot_user + self.rotList[0]
            del self.posList[0]
            del self.rotList[0]

        return return_pos, return_rot

    def CreateNoiseDataSet(self, frequency, scale):
        noisePos = []
        noiseRot = []
        noisePos_filt = []
        noiseRot_filt = []

        for i in range(3):
            noisePos.append(np.random.normal(loc = 0, scale = scale, size = 1800))
            noiseRot.append(np.random.normal(loc = 0, scale = scale, size = 1800))

        self.motionFilter.InitLowPassFilterWithOrder(samplerate = 180, fp = frequency, n = 2)

        for j in range(3):
            noisePos_filt.append(self.motionFilter.ButterFilter(noisePos[j]))
            noiseRot_filt.append(self.motionFilter.ButterFilter(noiseRot[j]))

        noisePos_filt = np.transpose(np.array(noisePos_filt))
        noiseRot_filt = np.transpose(np.array(noiseRot_filt))

        return noisePos_filt, noiseRot_filt

    def ConvertToIterator(self, ListConverted):
        return iter(ListConverted)

    def ConvertToIteratorCycle(self, ListToConverted):
        return itertools.cycle(ListToConverted)

if __name__ == '__main__':
    noiseMotion = NoiseMotion(Frequency = 10, Scale = 10, UserControlWeight = 0.5)
    while True:
        print(next(noiseMotion.noiseSet_Pos_Iter))
