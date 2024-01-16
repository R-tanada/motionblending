import numpy as np

def jet(m):
    n = round(m / 4)
    u = np.concatenate([np.linspace(0, 1, n), np.ones(n-1), np.linspace(1, 0, n)])
    print(u)
    g = np.maximum(0, 1 - np.abs(1 - 2 * u))
    r = np.maximum(0, 1 - np.abs(1 - 2 * u[2*n:3*n]))
    b = np.maximum(0, 1 - np.abs(1 - 2 * u[:n]))
    print(len(r), len(g), len(b))
    jet_map = np.column_stack((r, g, b))
    return jet_map

# 例: カラーマップサイズが256の場合
m = 256
jet_colormap = jet(m)