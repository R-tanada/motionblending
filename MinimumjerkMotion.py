import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize

# 経由点を指定して最小躍度軌道を求める関数
def compute_minimum_jerk_path(waypoints, times):
    num_waypoints = len(waypoints)
    num_variables = 3 * num_waypoints  # 各経由点の位置（x, y, z）を変数とする

    # 最小躍度を最適化するための目的関数
    def objective(x):
        return np.linalg.norm(np.diff(np.diff(x.reshape(num_waypoints, 3), axis=0), axis=0))  # 躍度のL2ノルム

    # 制約条件関数
    def constraints(x):
        return np.concatenate([
            x[:3] - waypoints[0],  # 初期位置の制約
            x[-3:] - waypoints[-1],  # 最終位置の制約
            np.diff(x.reshape(num_waypoints, 3), axis=0).flatten() - np.diff(waypoints, axis=0).flatten(),  # 経由点の制約
            np.dot(A, x.reshape(num_waypoints, 3).T).T.flatten() - b  # その他の制約（例えば速度や加速度の制約）
        ])

    # 制約行列とベクトルを設定
    A = np.zeros((0, num_variables))
    b = np.zeros(0)

    # 最適化の初期値
    x0 = np.zeros(num_variables)

    # 最適化を実行
    result = minimize(objective, x0, constraints={'type': 'eq', 'fun': constraints})

    if result.success:
        # 最適解から位置の軌道を計算
        solution = result.x.reshape(num_waypoints, 3).T

        # 時間に応じた補間関数を作成
        interp_func = interpolate.interp1d(times, solution, axis=1, kind='cubic')

        # 時間軸の範囲を指定
        t_min, t_max = times[0], times[-1]

        # 補間関数を用いて連続的な軌道を計算
        def path(t):
            return interp_func(t)

        return path

    else:
        print("Optimization failed.")
        return None

# 経由点と通過時刻の設定
waypoints = np.array([[0, 0, 0], [1, 1, 1], [2, 0, 2]])  # 経由点の位置
times = np.array([0, 1, 2])  # 経由点を通過する時刻

# 最小躍度軌道の計算
path = compute_minimum_jerk_path(waypoints, times)

# 時間に対応する位置を取得
t = np.linspace(times[0], times[-1], 100)
positions = path(t)

# 結果の表示
for i in range(len(t)):
    print(f"Time: {t[i]}, Position: {positions[:, i]}")
