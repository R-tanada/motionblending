import numpy as np
import random

class LatencyMotionFunction:
    def __init__(self, latency_time, UserControlWeight) -> None:
        self.posList = []
        self.rotList = []
        self.latency_time = latency_time
        self.controlWeigt = {'user':UserControlWeight, 'robot':1 - UserControlWeight}
        print(self.controlWeigt)
        print(self.latency_time)
        self.num = 0

    def JustLatecyFunc(self, position, rotation, current_time):
        pos_user = position * self.controlWeigt['user']
        pos_robot = position * self.controlWeigt['robot']
        rot_user = rotation * self.controlWeigt['user']
        rot_robot = rotation * self.controlWeigt['robot']

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
        pos_user = position * self.controlWeigt['user']
        pos_robot = position * self.controlWeigt['robot'] + (10 * random.random())
        rot_user = rotation * self.controlWeigt['user']
        rot_robot = rotation * self.controlWeigt['robot'] + (10 * random.random())

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

    def RealtimeNoise(self, position,rotation):
        return position + (10 * random.random()), rotation + (10 * random.random())

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
