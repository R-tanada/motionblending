import numpy as np
from scipy.optimize import minimize

# 目的関数（躍度）
def jerk(x):
    j = np.sum(np.abs(np.gradient(np.gradient(np.gradient(x)))))
    return j

# 制約条件関数
def constraint(x, *args):
    p1, p2, p3 = args
    c1 = np.dot(x[:3] - p1, x[3:])  # 第1の経由点
    c2 = np.dot(x[:3] - p2, x[3:])  # 第2の経由点
    c3 = np.dot(x[:3] - p3, x[3:])  # 第3の経由点
    return [c1, c2, c3]

# 躍度最小軌道の導出
def optimize_path(init_path, p1, p2, p3):
    bounds = [(None, None)] * 3 + [(-1, 1)] * 3
    res = minimize(jerk, init_path, constraints={'type': 'eq', 'fun': constraint, 'args': (p1, p2, p3)}, bounds=bounds, method='SLSQP')
    return res.x

# 例として、初期値となる軌道を与える
init_path = np.array([0, 0, 0, 1, 0, 0, 0])

# 3つの経由点を与える
p1 = np.array([1, 2, 3])
p2 = np.array([4, 5, 6])
p3 = np.array([7, 8, 9])

# 躍度最小軌道を求める
result_path = optimize_path(init_path, p1, p2, p3)

print(result_path)
