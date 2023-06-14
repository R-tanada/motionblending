import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

data = []

with open('TestFile/data/mocap_raw_data.csv') as f:
    reader = csv.reader(f)
    data = [row for row in reader][1:]                  # Remove header
    data = [[float(v) for v in row] for row in data]    # Convert to float

data_iter = iter(data)

# プロットの初期化
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# データを保持するリストを作成
x_data, y_data, z_data = [], [], []

# プロットの更新処理
def update_plot(x, y, z):
    # データを追加
    x_data.append(x)
    y_data.append(y)
    z_data.append(z)

    # データをクリアし、新しいデータでプロットを更新
    ax.clear()
    ax.plot(x_data, y_data, z_data)
    
    # 軸の範囲を設定（必要に応じて調整）
    ax.set_xlim3d(-10, 10)
    ax.set_ylim3d(-10, 10)
    ax.set_zlim3d(-10, 10)

    # グラフを再描画
    plt.draw()

# ダミーデータの生成と更新処理の実行（ここではランダムなデータを生成しています）
while True:
    data = next(data_iter)
    x = data[0]
    y = data[1]
    z = data[2]
    
    update_plot(x, y, z)
    plt.pause(0.1)  # プロットを更新する間隔（秒）

# プロットの表示
plt.show()
