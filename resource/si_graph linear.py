import csv
import numpy as np
from matplotlib import pyplot as plt

path = '/Users/yuzu/Documents/GitHub/MotionBlending-CA/resource/linear/pos20231212_102939.csv'

with open(path) as file:
    reader = csv.reader(file)
    data = [row for row in reader][1:]
    data = [[float(v) for v in row] for row in data]
    data = np.array(data)

t0 = 0
tf = 2.0228033468271223
tp = 0.8504217000000001
x0 = np.array([10.23212075, -35.10066867, 5.64062595])
xf = np.array([302.543274, -184.999939, 64.862747])
target = xf - x0
xf_norm = np.sqrt(target[0]**2 + target[1]**2 + target[2]**2)

time = data[:, 0]
pos = data[:, 1:4]
pos = pos - pos[1, :]
norm = np.sqrt(pos[:, 0]**2 + pos[:, 1]**2 + pos[:, 2]**2)
user_pos = norm/xf_norm

time = time - time[0]
time = time/time[-1]

traject = xf_norm * np.array(time)/ xf_norm
print(len(traject), len(user_pos))

plt.plot(time, user_pos)
plt.plot(time, traject)
plt.show()



