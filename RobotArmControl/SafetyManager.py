from math import pi
import CustomFunction.Calculation as cf
import numpy as np

class SafetyManager:
    def __init__(self, xArmConfigs):
        self.Max_X, self.Max_Y, self.Max_Z, self.Max_Roll, self.Max_Pitch, self.Max_Yow = self.SetMaximum(xArmConfigs['MaxPos'], xArmConfigs['MaxRot'])
        self.Min_X, self.Min_Y, self.Min_Z, self.Min_Roll, self.Min_Pitch, self.Min_Yow = self.SetMinimum(xArmConfigs['MinPos'], xArmConfigs['MinRot'])

    def SetMaximum(self, MaxPos, MaxRot):
        return MaxPos[0], MaxPos[1], MaxPos[2], MaxRot[0], MaxRot[1], MaxRot[2]

    def SetMinimum(self, MinPos, MinRot):
        return MinPos[0], MinPos[1], MinPos[2], MinRot[0], MinRot[1], MinRot[2]

    def Check(self, x, y, z, roll, pitch, yaw):
        # pos X
        if(x > self.Max_X):
            x = self.Max_X
        elif(x < self.Min_X):
            x = self.Min_X

        # pos y
        if(y> self.Max_Y):
            y = self.Max_Y
        elif(y < self.Min_Y):
            y = self.Min_Y

        # pos z
        if(z > self.Max_Z):
            z = self.Max_Z
        elif(z < self.Min_Z):
            z = self.Min_Z

        # # roll
        # if(0 < rotation[0] < self.MaxRot[0]):
        #     rotation[0] = self.MaxRot[0]
        # elif(self.MinRot[0] < rotation[0] < 0):
        #     rotation[0] = self.MinRot[0]

        # # pitch
        # if(rotation[1] > self.MaxRot[1]):
        #     rotation[1] = self.MaxRot[1]
        # elif(rotation[1] < self.MinRot[1]):
        #     rotation[1] = self.MinRot[1]

        # # yaw
        # if(rotation[2] > self.MaxRot[2]):
        #     rotation[2] = self.MaxRot[2]
        # elif(rotation[2] < self.MinRot[2]):
        #     rotation[2] = self.MinRot[2]

        return [x, y, z, roll, pitch, yaw]

