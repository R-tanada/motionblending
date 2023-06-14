import csv
import numpy as np
from matplotlib import pyplot as plt
import time
from FilterManager import RealTimeLowpassFilter

data = []
data = []

with open('TestFile/data/mocap_raw_data.csv') as f:
    reader = csv.reader(f)
    data = [row for row in reader][1:]                  # Remove header
    data = [[float(v) for v in row] for row in data]    # Convert to float

data_iter = iter(data)

pos_list = []
vel_list = []
acc_list = []
pos_list2 = []
vel_list2 = []
acc_list2 = []
time_list = []
posBox = []
velBox = []
dt = 1/200

space = 5

poslist = []
vellist = []
acclist = []

def GetParticipantMotionInfo(position):
    global posBox ,dt
    posBox.append(np.linalg.norm(position))

    if len(posBox) == space:
        vel = (posBox[space - 1] - posBox[0])/ (dt * (space - 1))
        velBox.append(vel)

        if len(velBox) == space:
            acc = (velBox[space - 1] - velBox[0])/ (dt * (space - 1))
            del velBox[0]

        else:
            acc = 0

        del posBox[0]

    else:
        vel = 0
        acc = 0

    return vel, acc

def GetParticipantMotionInfo2(position):
    vel = acc = 0
    poslist.append(position)

    if len(poslist) == space:
        velocity = (poslist[space - 1] - poslist[0])/ (dt * (space -1))
        vellist.append(velocity)

        if len(vellist) == space:
            if np.linalg.norm(poslist[space - 1]) - np.linalg.norm(poslist[0]) >= 0:
                vel = np.linalg.norm(vellist[space - 1])

            else:
                vel = -np.linalg.norm(vellist[space - 1])

            acceleration = (vellist[space - 1] - vellist[0])/ (dt * (space -1))
            acclist.append(acceleration)

            if len(acclist) == space:
                if np.linalg.norm(vellist[space - 1]) - np.linalg.norm(vellist[0]) >= 0:
                    acc = np.linalg.norm(acclist[space - 1])

                else:
                    acc = -np.linalg.norm(acclist[space - 1])

                del acclist[0]

            else:
                acc = 0
            
            del vellist[0]

        else:
            vel = 0

        del poslist[0]

    else:
        vel = 0
        acc = 0

    return vel, acc

# vel = [0, 0, 0]
# acc = [0, 0, 0]

filter = RealTimeLowpassFilter(cutoff_freq=1, fs=200, order=2, listNum=3)

try:
    while True:
        inputdata = next(data_iter)
        position = filter.apply(inputdata[0:3])
        
        vel, acc = GetParticipantMotionInfo(position)
        vel2, acc2 = GetParticipantMotionInfo2(position)
        # print(acc)

        # pos_list.append(pos)
        # vel_list.append(vel)
        # vel_list2.append(vel2)
        acc_list.append(acc)
        acc_list2.append(acc2)
        time_list.append(inputdata[3])

        time.sleep(0.005)

except StopIteration:
    fig = plt.figure()
    plot = fig.add_subplot()
    plot.set_xlabel('time')
    plot.set_ylabel('velocity')
    plot.plot(time_list, acc_list, label = 'siraki')
    plot.plot(time_list, acc_list2, label = 'honnrai')
    plot.legend()
    plt.show()



