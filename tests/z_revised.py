import numpy as np
from matplotlib import pyplot as plt

t = np.linspace(0,1,300)
y = np.linspace(0,1,300)

for i in range(len(t)):
    if t[i] > 0.8:
        w = (t[i] - 0.8)/0.2
        y[i] += np.sin(0.5*np.pi *w)

print(t)
plt.plot(t, y)
plt.show()
