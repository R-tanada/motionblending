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

class Fitting:
    def __init__(fitting, path) -> None:
        data = fitting.load(path)
        time = data[:, 0]
        time = time - time[0]
        time = time / time[-1]
        pos = data[:, 1:4]
        pos = pos - pos[0]
        norm = np.sqrt(pos[:, 0] ** 2 + pos[:, 1] ** 2 + pos[:, 2] ** 2)
        norm = norm / norm[-1]

        fitting.coe = fitting.custom_fit(time, norm)

    def load(fitting, path):
        with open(path) as file:
            reader = csv.reader(file)
            data = [row for row in reader][1:]
            data = [[float(v) for v in row] for row in data]
            data = np.array(data)

        return data

    def custom_fit(fitting, x, y):
        a11 = sum((x**5 - x) ** 2)
        a12 = sum((x**5 - x) * (x**4 - x))
        a13 = sum((x**5 - x) * (x**3 - x))
        a14 = sum((x**5 - x) * (x**2 - x))
        a21 = sum((x**4 - x) * (x**5 - x))
        a22 = sum((x**4 - x) ** 2)
        a23 = sum((x**4 - x) * (x**3 - x))
        a24 = sum((x**4 - x) * (x**2 - x))
        a31 = sum((x**3 - x) * (x**5 - x))
        a32 = sum((x**3 - x) * (x**4 - x))
        a33 = sum((x**3 - x) ** 2)
        a34 = sum((x**3 - x) * (x**2 - x))
        a41 = sum((x**2 - x) * (x**5 - x))
        a42 = sum((x**2 - x) * (x**4 - x))
        a43 = sum((x**2 - x) * (x**3 - x))
        a44 = sum((x**2 - x) ** 2)

        A = np.array(
            [
                [a11, a12, a13, a14],
                [a21, a22, a23, a24],
                [a31, a32, a33, a34],
                [a41, a42, a43, a44],
            ]
        )
        A_inv = np.linalg.inv(A)

        b1 = sum((y - x) * (x**5 - x))
        b2 = sum((y - x) * (x**4 - x))
        b3 = sum((y - x) * (x**3 - x))
        b4 = sum((y - x) * (x**2 - x))

        B = np.array([b1, b2, b3, b4])

        return np.dot(A_inv, B)


path = '/Users/yuzu/Documents/GitHub/MotionBlending-CA/resource/personal/pos20231212_113157.csv'

with open(path) as file:
    reader = csv.reader(file)
    data = [row for row in reader][1:]
    data = [[float(v) for v in row] for row in data]
    data = np.array(data)

t0 = 0
tf = 2.6241200880578734
tp = 0.7354852999999988/tf
x0 = np.array([  4.64791059 ,-37.59099543  ,19.39898729])
xf = np.array([ 302.543274, -184.999939  , 64.862747])
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
time_n = time_n_1 - time_n_1[0]
time_n = time_n/time_n[-1]

pos_n = user_pos[index_tp:]


fitting = Fitting('/Users/yuzu/Documents/GitHub/MotionBlending-CA/resource/SI2023/tanada20231211_180827.csv')

y_fit = fitting.coe[0] * time ** 5 + fitting.coe[1] * time ** 4 + fitting.coe[2] * time ** 3 + fitting.coe[3] * time ** 2 + (1 - (fitting.coe[0] + fitting.coe[1] + fitting.coe[2] + fitting.coe[3])) * time

y_fit_n = y_fit[index_tp:]

combined1 = [x * y for (x, y) in zip(pos_n, 1-time_n)]
print(combined1[-1])
combined2 = [x * y for (x, y) in zip(y_fit_n, time_n)]
print(combined2[-1])
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

with open('/Users/yuzu/Desktop/M2/修論/matlab/' + 'personal_1.csv', 'w', newline='') as exportFile:
    writer = csv.writer(exportFile)
    writer.writerows(data_1)

with open('/Users/yuzu/Desktop/M2/修論/matlab/' + 'personal_2.csv', 'w', newline='') as exportFile:
    writer = csv.writer(exportFile)
    writer.writerows(data_2)

# plt.show()


