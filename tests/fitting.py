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

data = load('/Users/yuzu/Documents/GitHub/MotionBlending-CA/resource/pos20230823_181927.csv')[0:500, 0]
time = load('/Users/yuzu/Documents/GitHub/MotionBlending-CA/resource/time20230823_181927.csv')[0:500, 0]

plt.plot(time, data)
plt.show()
print(time)
 
# ばらつきを持った3次関数の波形を生成
a1 = -2
a2 = 1
a3 = 20
a4 = 1
x = np.arange(-5, 5, 0.2)                              # 時間軸配列を作成
noise = np.random.normal(loc=0, scale=10, size=len(x)) # ガウシアンノイズを生成
y = a1 * x ** 3 + a2 * x ** 2 + a3 * x + a4 + noise    # 3次関数にノイズを重畳
 
# 近似パラメータakを算出
coe = np.polyfit(x, y, 3)
print(coe)
 
# 得られたパラメータakからカーブフィット後の波形を作成
y_fit = coe[0] * x ** 3 + coe[1] * x ** 2 + coe[2] * x + coe[3]

# plt.plot(x, y_fit)
# plt.show()