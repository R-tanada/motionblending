import numpy as np
from scipy.optimize import minimize
from matplotlib import pyplot as plt

# 制約条件
def constraint(x):
    # 経由点を通る制約式
    c = 50 - x[50]
    return c

# 躍度関数
def jerk(x):
    j = np.sum(np.abs(np.gradient(np.gradient(np.gradient(x)))))
    return j

# 最適化問題を設定
def optimize_path(init_path, p1, p2, p3):
    res = minimize(jerk, init_path, constraints = {'type': 'eq', 'fun': constraint}, method='SLSQP')
    return res.x

# 例として、初期値となる軌道を与える
init_path = np.linspace(100, 50, 100)

# 3つの経由点を与える
p1 = 2
p2 = 4
p3 = 19

# 経由点を制約条件として、最適な軌道を求める
result_path = optimize_path(init_path, p1, p2, p3)

plt.plot(init_path, result_path)
plt.show()

# print(result_path)
