import random
import numpy as np
from matplotlib import pyplot as plt

from MotionFilter.MotionFilter import MotionFilter

class LatencyMotion:
    def __init__(self, latency_time, UserControlWeight) -> None:
        self.posList = []
        self.rotList = []
        self.latency_time = latency_time
        self.controlWeigt = {'user':UserControlWeight, 'robot':1 - UserControlWeight}
        self.num = 0

    def JustLatecyFunc(self, position, rotation, current_time):
        print(position)
        pos_user = np.array(position) * self.controlWeigt['user']
        pos_robot = np.array(position) * self.controlWeigt['robot']
        rot_user = np.array(rotation) * self.controlWeigt['user']
        rot_robot = np.array(rotation) * self.controlWeigt['robot']

        self.posList.append(pos_robot)
        self.rotList.append(rot_robot)

        if current_time < self.latency_time:
            return_pos = pos_user
            return_rot = rot_user
        else:
            return_pos = pos_user + self.posList[0]
            return_rot = rot_user + self.rotList[0]
            del self.posList[0]
            del self.rotList[0]

        return return_pos, return_rot

    def NoiseFunc(self, position, rotation, current_time):
        pos_user = np.array(position) * self.controlWeigt['user']
        pos_robot = np.array(position) * self.controlWeigt['robot'] + (1 * random.random())
        rot_user = np.array(rotation) * self.controlWeigt['user']
        rot_robot = np.array(rotation) * self.controlWeigt['robot'] + (1 * random.random())

        self.posList.append(pos_robot)
        self.rotList.append(rot_robot)

        if current_time < self.latency_time:
            return_pos = pos_user
            return_rot = rot_user
        else:
            return_pos = pos_user + self.posList[0]
            return_rot = rot_user + self.rotList[0]
            del self.posList[0]
            del self.rotList[0]

        return return_pos, return_rot

    def RealtimeNoise(self, position, rotation):
        return np.array(position) + (2 * random.random()), np.array(rotation) + (2 * random.random())

    def InverseFunc(self, position, rotation, current_time):
        pos_user = position * self.controlWeigt['user']
        pos_robot = position * self.controlWeigt['robot'] * -1
        rot_user = rotation * self.controlWeigt['user']
        rot_robot = rotation * self.controlWeigt['robot'] * -1

        self.posList.append(pos_robot)
        self.rotList.append(rot_robot)

        if current_time < self.latency_time:
            return_pos = pos_user
            return_rot = rot_user
        else:
            return_pos = pos_user + self.posList[0]
            return_rot = rot_user + self.rotList[0]
            del self.posList[0]
            del self.rotList[0]

        return return_pos, return_rot
    
if __name__ == '__main__':
    latMot = LatencyMotion(3, 0.5)
    motionFilter = MotionFilter()
    motionFilter.InitLowPassFilterWithOrder(180, 3, 2)
    

    x = np.arange(0, 1800) / 180
    data = np.random.normal(loc=0, scale=10, size=len(x))
    data_lowpass = motionFilter.ButterFilter(data)

    fig = plt.figure(figsize = (10,6), facecolor='lightblue')

    plot_raw = fig.add_subplot(2, 1, 1)
    plot_lowpass = fig.add_subplot(2, 1, 2)

    plot_raw.plot(x, data)
    plot_lowpass.plot(x, data_lowpass)

    plt.show()

    

