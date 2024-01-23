import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import csv

# 最小化する目的関数の定義
def objective_function(coefficients, x, y):
    # 多項式の評価
    predicted_y = np.polyval(coefficients, x)
    # 最小二乗誤差を計算
    mse = np.mean((predicted_y - y)**2)
    return mse

# 初期値と終端値に対する制約条件の定義
def constraint(coefficients, x_initial, y_initial, x_terminal, y_terminal):
    # 初期値の制約
    initial_constraint = np.polyval(coefficients, x_initial) - y_initial
    # 終端値の制約
    terminal_constraint = np.polyval(coefficients, x_terminal) - y_terminal
    return np.array([initial_constraint, terminal_constraint])


with open('resource/iida/model_data/left20240122_090519.csv') as file:
    reader = csv.reader(file)
    data = [row for row in reader][1:]
    data = [[float(v) for v in row] for row in data]
    data = np.array(data)

time = data[:, 0]
norm = np.sqrt(data[:, 1]**2 + data[:, 2]**2 + data[:, 3])

time = time - time[0]
norm = norm - norm[0]

time = time/time[-1]
norm = norm/norm[-1]

x_data = time
y_data = norm

# 初期値と終端値
x_initial, y_initial = 1, 1
x_terminal, y_terminal = 0, 0

# 最適化の初期値
initial_coefficients = np.ones(5)  # 5次の多項式を仮定

# 最小化の実行
result = minimize(
    objective_function,
    initial_coefficients,
    args=(x_data, y_data),
    constraints={'type': 'eq', 'fun': constraint, 'args': (x_initial, y_initial, x_terminal, y_terminal)}
)

# 結果表示
print("最適な係数:", result.x)

# 得られた係数
optimal_coefficients = result.x

# データと多項式近似のプロット
plt.scatter(x_data, y_data, label='Data')
x_range = np.linspace(min(x_data), max(x_data), 100)
y_approx = np.polyval(optimal_coefficients, x_range)
plt.plot(x_range, y_approx, color='red', label='Polynomial Approximation')

# 初期値と終端値の点を強調
plt.scatter([x_initial, x_terminal], [y_initial, y_terminal], color='green', label='Initial and Terminal Points')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Polynomial Approximation with Constraints')
plt.legend()
plt.show()
