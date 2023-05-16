import time

def FixFrameRate(processDuration):
    sleepTime = self.loopTime - processDuration
    if sleepTime < 0:
        pass
    else:
        time.sleep(sleepTime)

def CheckFrameRate(loopTime):
    self.FrameList.append(1/ loopTime)
    if len(self.FrameList) == 30:
        print(sum(self.FrameList)/ len(self.FrameList))
        self.FrameList = []