import numpy as np
from scipy.optimize import minimize
from matplotlib import pyplot as plt

# 躍度最小化の目的関数
def objective(x):
    return np.sum(np.abs(np.gradient(np.gradient(np.gradient(x)))))

# 初期値と終端地点
initial_position = 0  # 初期位置
final_position = 1    # 終端位置

def cons(x):
    return np.sum(np.gradient(np.gradient(np.gradient(np.gradient(np.gradient(np.gradient(x)))))))

# 初期軌道の数
num_samples = 200

# 初期軌道の生成
initial_trajectory = np.linspace(initial_position, final_position, num_samples)

# 制約条件
constraints = [{'type': 'eq', 'fun': lambda x: x[0] - initial_position},   # 初期値の制約
               {'type': 'eq', 'fun': lambda x: x[-1] - final_position},
               {'type': 'eq', 'fun': cons}]    # 終端地点の制約

# 最適化の実行
result = minimize(objective, initial_trajectory, constraints=constraints)

# 予測された手の動き
hand_movement = result.x

# 結果の表示
plt.plot(initial_trajectory, result.x)
plt.show()
