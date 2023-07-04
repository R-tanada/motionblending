import numpy as np
from scipy.optimize import minimize
from matplotlib import pyplot as plt
i = 0

# 躍度を計算する関数
def jerk(x):
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    return np.sum(np.abs(np.gradient(ddx))**2)

# 最小化する関数（躍度の総和）
def objective_function(x):
    global i
    print(i)
    i += 1
    return jerk(x)

def constraint_1(x):
    return x[0]

def constraint_2(x):
    return x[-1]-100

def constraint_3(x):
    dx = np.gradient(x)
    return dx[0]

def constraint_4(x):
    dx = np.gradient(x)
    return dx[-1]

def constraint_5(x):
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    return ddx[0]

def constraint_6(x):
    dx = np.gradient(x)
    ddx = np.gradient(dx)
    return ddx[-1]

def constraint_7(x):
    return np.diff(x)

constraints = [
    {'type': 'eq', 'fun': constraint_1},  # 等式制約1
    {'type': 'eq', 'fun': constraint_2},  # 等式制約2
    {'type': 'eq', 'fun': constraint_3}, 
    {'type': 'eq', 'fun': constraint_4},  # 等式制約1
    {'type': 'eq', 'fun': constraint_5},  # 等式制約2
    {'type': 'eq', 'fun': constraint_6}, 
    {'type': 'ineq', 'fun': constraint_7}
]

bounds = [(0, 100)] * 200  # 共通の範囲を指定


# 初期軌道
# initial_trajectory = np.random.rand(200)
initial_trajectory = np.zeros(200)

# 最小化問題の解を取得
res = minimize(objective_function, initial_trajectory, method='SLSQP', constraints=constraints, bounds=bounds)

# 最適解の値
optimal_trajectory = res.x

print(optimal_trajectory)



plt.plot(np.linspace(0, 1, len(optimal_trajectory)), optimal_trajectory)
# plt.plot(np.linspace(0, 1, len(optimal_trajectory)), initial_trajectory)
plt.show()