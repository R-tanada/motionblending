import csv
import numpy as np
from matplotlib import pyplot as plt

def getNearestValue(list, num):
    """
    概要: リストからある値に最も近い値を返却する関数
    @param list: データ配列
    @param num: 対象値
    @return 対象値に最も近い値
    """

    # リスト要素と対象値の差分を計算し最小値のインデックスを取得
    idx = np.abs(np.asarray(list) - num).argmin()
    return idx

path = '/Users/yuzu/Documents/GitHub/MotionBlending-CA/resource/linear/pos20231212_113006.csv'

with open(path) as file:
    reader = csv.reader(file)
    data = [row for row in reader][1:]
    data = [[float(v) for v in row] for row in data]
    data = np.array(data)

t0 = 0
tf = 1.845111466992304
tp = 0.8005890000000004/tf
x0 = np.array([ 16.70654863, -41.23340547 , 19.47200298])
xf = np.array([ 302.543274, -184.999939,   64.862747])
target = xf - x0
xf_norm = np.sqrt(target[0]**2 + target[1]**2 + target[2]**2)

time = data[:, 0]
pos = data[:, 1:4]
pos = pos - pos[1, :]
norm = np.sqrt(pos[:, 0]**2 + pos[:, 1]**2 + pos[:, 2]**2)
user_pos = norm/xf_norm

time = time - time[0]
time = time/time[-1]

index_tp = getNearestValue(time, tp)

time_n_1 = time[index_tp:]
time_n = (time_n_1 - time_n_1[0])
time_n = time_n/time_n[-1]

pos_n = user_pos[index_tp:]

y_fit = time

y_fit_n = y_fit[index_tp:]

combined1 = [x * y for (x, y) in zip(pos_n, 1-time_n)]
combined2 = [x * y for (x, y) in zip(y_fit_n, time_n)]
y_robot = [x + y for (x, y) in zip(combined1, combined2)]
# print(y_robot)

print(len(y_robot), len(time_n))

plt.plot(time, user_pos, label = 'user', linewidth=3)
plt.plot(time, y_fit, label = 'traject', linewidth=3)
plt.plot(time_n_1, y_robot, label = 'robot', linewidth=3)
plt.vlines(tp, 0, 0.7, color='k', linestyles='dotted')

plt.legend(loc = 'upper left', fontsize = 15) 
plt.xlabel('time', fontsize = 15)
plt.ylabel('norm', fontsize = 15)
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)

data_1 = np.array([time, user_pos, y_fit]).transpose()
data_2 = np.array([time_n_1, y_robot]).transpose()

with open('/Users/yuzu/Desktop/M2/修論/matlab/' + 'linear_1.csv', 'w', newline='') as exportFile:
    writer = csv.writer(exportFile)
    writer.writerows(data_1)

with open('/Users/yuzu/Desktop/M2/修論/matlab/' + 'linear_2.csv', 'w', newline='') as exportFile:
    writer = csv.writer(exportFile)
    writer.writerows(data_2)

# plt.show()


