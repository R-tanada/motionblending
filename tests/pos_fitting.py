import csv

import numpy as np
from matplotlib import pyplot as plt


def load(path):
    with open(path) as file:
        reader = csv.reader(file)
        data = [row for row in reader][1:]
        data = [[float(v) for v in row] for row in data]
        data = np.array(data)

    return data


def custom_fit(x, y):
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


data = load("resource\hanai\model_data\left20231212_130834.csv")
time = data[:, 0]
time = time - time[0]
time = time / time[-1]
pos = data[:, 1:4]
pos = pos - pos[0]
norm = np.sqrt(pos[:, 0] ** 2 + pos[:, 1] ** 2 + pos[:, 2] ** 2)
norm = norm / norm[-1]

coe = custom_fit(time, norm)
print(coe)

y_fit = (
    coe[0] * time**5
    + coe[1] * time**4
    + coe[2] * time**3
    + coe[3] * time**2
    + (1 - (coe[0] + coe[1] + coe[2] + coe[3])) * time
)

# 近似パラメータakを算出
# coe = np.polyfit(time, norm, 5)
# print(coe)

# 得られたパラメータakからカーブフィット後の波形を作成
# y_fit = coe[0] * time ** 4 + coe[1] * time ** 3 + coe[2] * time ** 2 + coe[3] * time + coe[4]
# y_fit = coe[0] * time ** 5 + coe[1] * time ** 4 + coe[2] * time ** 3 + coe[3] * time **2 + coe[4] * time + coe[5]

plt.plot(time, norm)
plt.plot(time, y_fit)
plt.show()
