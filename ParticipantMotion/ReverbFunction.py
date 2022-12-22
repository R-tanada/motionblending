import numpy as np

class ReverbFunction:
    def __init__(self, echoNum, echoWidth, function, min, max) -> None:
        # self.latencyList = self.CreateLatencyList(echoNum, echoWidth)
        # self.userMotionRate, self.echoRateList = self.CreateLinerMotionRate(echoNum)
        self.latencyList = self.CreateLatencyList_2(echoNum, echoWidth)
        self.userMotionRate, self.echoRateList = self.CreateCustomMotionRate(self.latencyList, min, max, function)
        self.latencyList = np.delete(self.latencyList, 0)
        print('userweight:{}, motionrate:{}, echotime:{}'.format(self.userMotionRate, self.echoRateList, self.latencyList))
        self.num = 0
        self.sumPos = 0
        self.sumRot = 0
        self.recordedPos = {}
        self.recordedRot = {}
        for i in range(echoNum):
            self.recordedPos['motion_%s'%i] = []
            self.recordedRot['motion_%s'%i] = []

    def CreateLatencyList(self, echoNum, echoWidth):
        max = echoNum * echoWidth
        latencyList = np.linspace(0, max, echoNum + 1)
        latencyList = np.delete(latencyList, 0)
        return latencyList

    def CreateLinerMotionRate(self, echoNum):
        motionlist = 1 - np.linspace(0, 1, echoNum + 2)
        motionlist = motionlist/np.sum(motionlist)
        motionlist = np.delete(motionlist, -1)
        userWeight = motionlist[0]
        echoRate = motionlist[1:]
        return userWeight, echoRate

    def CreateMotionRateWithInverse(self, echoNum):
        def inverse(x):
            return 1/x

        list = np.linspace(1, 20, echoNum)
        motionlist = inverse(list)
        print(motionlist)
        return motionlist

    def motionFunc(self, position, rotation, time):
        self.userPosition = position * self.userMotionRate
        self.userRotation = rotation * self.userMotionRate

        for i in range(len(self.echoRateList)):
            self.recordedPos['motion_%s'%i].append(position * self.echoRateList[i])
            self.recordedRot['motion_%s'%i].append(rotation * self.echoRateList[i])

        if time >= self.latencyList[self.num]:
            print(self.num)
            for n in range(self.num + 1):
                self.sumPos += self.recordedPos['motion_%s'%n][0]
                self.sumRot += self.recordedRot['motion_%s'%n][0]
                del self.recordedPos['motion_%s'%n][0]
                del self.recordedRot['motion_%s'%n][0]

                if len(self.latencyList) - 1 > self.num:
                    if time >= self.latencyList[self.num + 1]:
                        self.num += 1
                    else:
                        pass
            return_pos = self.userPosition + self.sumPos
            return_rot = self.userRotation + self.sumRot
            self.sumPos = 0
            self.sumRot = 0

        else:
            return_pos = self.userPosition
            return_rot = self.userRotation

        return return_pos, return_rot

    def CreateLatencyList_2(self, echoNum, echoWidth):
        max = echoNum * echoWidth
        latencyList = np.linspace(0, max, echoNum + 1)
        return latencyList

    def CreateCustomMotionRate(self, latencyList, minNum, maxNum, function):
        def liner(x):
            x = np.array(x)
            return np.max(x) - x

        def multiply(x):
            x = np.array(x)
            return 2 ** (-x)

        def squared(x):
            y = x ** 2
            return max(y) - y

        def squared_sin(x):
            return np.sin(x) ** 2

        def normalization(motionList):
            return motionList/sum(motionList)

        def MappingList(list, min, max):
            list = np.array(list)
            return ((list/np.max(list)) * (max - min)) + min

        mappinglist = MappingList(latencyList, minNum, maxNum)

        if function == 'liner':
            motionlist = liner(mappinglist)
        elif function == 'squared':
            motionlist = squared(mappinglist)
        elif function == 'squared_sin':
            motionlist = squared_sin(mappinglist)
        elif function == 'multiply':
            motionlist = multiply(mappinglist)

        motionrate = normalization(motionlist)
        return motionrate[0], motionrate[1:]

if __name__ == '__main__':
    reverbFunc = ReverbFunction(echoNum = 6, echoWidth = 0.2, function = 'multiply', min = 0, max = 10)

