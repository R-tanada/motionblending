import numpy as np
from scipy.optimize import minimize
from matplotlib import pyplot as plt

# dt = 1/200

def constraint(x):
    c1 = 200 - x[-1]
    c2 = 100 - x[0]
    c3 = np.gradient(x)[0] - 6
    c4 = np.gradient(x)[-1]
    # c5 = np.gradient(np.gradient(x))[0] - 1
    c6 = np.gradient(np.gradient(x))[-1]
    return [c1, c2, c3, c4, c6]

def jerk(x):
    j = np.sum(np.abs(np.gradient(np.gradient(np.gradient(x)))))
    return j

def optimize_path(init_path):
    res = minimize(jerk, init_path, constraints = {'type': 'eq', 'fun': constraint})
    return res.x

# 例として、初期値となる軌道を与える
init_path = np.linspace(100, 200, 30)

# 経由点を制約条件として、最適な軌道を求める
result_path = optimize_path(init_path)

plt.plot(init_path, result_path)
plt.show()

# print(result_path)
