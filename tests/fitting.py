import numpy as np
from matplotlib import pyplot as plt
import csv

def load(path):
    with open(path) as file:
        reader = csv.reader(file)
        data = [row for row in reader][1:]
        data = [[float(v) for v in row] for row in data]
        data = np.array(data)

    return data

def custom_ployfit(x, y):
    a11 = sum((x**4 - x)**2)
    a12 = sum((x**4 - x)*(x**3 - x))
    a13 = sum((x**4 - x)*(x**2 - x))
    a21 = sum((x**4 - x)*(x**3 - x))
    a22 = sum((x**3 - x)**2)
    a23 = sum((x**3 - x)*(x**2 - x))
    a31 = sum((x**4 - x)*(x**2 - x))
    a32 = sum((x**3 - x)*(x**2 - x))
    a33 = sum((x**2 - x)**2)
    A = np.array([
        [a11, a12, a13],
        [a21, a22, a23],
        [a31, a32, a33]
    ])
    A_inv = np.linalg.inv(A)

    b1 = sum(y*(x**4 - x))
    b2 = sum(y*(x**3 - x))
    b3 = sum(y*(x**2 - x))
    B = np.array([
        b1, 
        b2,
        b3
    ])

    return np.dot(A_inv, B)

data = load('/Users/yuzu/Documents/GitHub/MotionBlending-CA/resource/velocity20231004_143223.csv')[:380]
time = data[:, 0] - data[0, 0]
time = time/time[-1]
velocity = data[:, 1]

coe = custom_ployfit(time, velocity)

y_fit = coe[0] * time ** 4 + coe[1] * time ** 3 + coe[2] * time ** 2 - (coe[0] + coe[1] + coe[2]) * time
 
# 近似パラメータakを算出
# coe = np.polyfit(time, velocity, 4)
print(coe)
 
# 得られたパラメータakからカーブフィット後の波形を作成
# y_fit = coe[0] * time ** 4 + coe[1] * time ** 3 + coe[2] * time ** 2 + coe[3] * time + coe[4]

plt.plot(time, velocity)
plt.plot(time, y_fit)
plt.show()