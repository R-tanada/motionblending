import numpy as np
from matplotlib import pyplot as plt
 
# ばらつきを持った2次関数の波形を生成
a1 = 1
a2 = 1
a3 = 1
x = np.arange(-5, 5, 0.2)                             # 時間軸配列を作成
noise = np.random.normal(loc=0, scale=2, size=len(x)) # ガウシアンノイズを生成
y = a1 * x ** 2 + a2 * x + a3 + noise                 # 2次関数にノイズを重畳
 
# 近似パラメータakを算出
coe = np.polyfit(x, y, 2)
print(coe)
 
# 得られたパラメータakからカーブフィット後の波形を作成
y_fit = coe[0] * x ** 2 + coe[1] * x + coe[2]