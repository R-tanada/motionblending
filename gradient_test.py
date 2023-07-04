import numpy as np
from matplotlib import pyplot as plt

t = np.linspace(0, np.pi, 200)

y = np.sin(t)

vel = np.gradient(y, 1/200)
acc = np.gradient(vel, 1/200)
jerk = np.gradient(acc, 1/200)

# print(b)


plt.plot(t, y)
plt.plot(t, vel)
plt.plot(t, acc)
plt.plot(t, jerk)
plt.show()