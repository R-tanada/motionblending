import csv
import numpy as np
from matplotlib import pyplot as plt
import time

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
time_list = []
posBox = []
dt = 1/200

def GetParticipnatMotionInfo(position):
    global posBox ,dt
    position = np.linalg.norm(position)
    posBox.append(position)

    if len(posBox) == 10:
        vel = (posBox[9] - posBox[0])/ (dt*10)
        acc = 0

    # if len(posBox) == 7:
    #     vel = (posBox[6] - posBox[3])/ (dt * 3)
    #     acc = (vel - ((posBox[3] - posBox[0])/ (dt * 3)))/ (dt * 3)
        del posBox[0]

    else:
        vel = acc = 0

    return position, vel, acc

try:
    while True:
        inputdata = next(data_iter)
        position = inputdata[0]
        
        pos, vel, acc = GetParticipnatMotionInfo(position)

        pos_list.append(pos)
        vel_list.append(vel)
        acc_list.append(acc)
        time_list.append(inputdata[3])

        time.sleep(0.005)

except StopIteration:
    plt.plot(time_list, pos_list)
    plt.plot(time_list, vel_list)
    plt.show()



