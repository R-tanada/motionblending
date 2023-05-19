from scipy.optimize import minimize
import numpy as np
from matplotlib import pyplot as plt

init_traject = np.linspace(0, 300, 500)
dt = 1/200

def function(x):
    jerk = np.gradient(np.gradient(np.gradient(x, dt), dt), dt)

    return jerk

def constraint(x):
    c1 = x[-1] - 300
    c2 = x[0] - 100

    return [c1, c2]

result = minimize(function, init_traject, constraints = {'fun': constraint, 'type': 'eq'}, method = 'SLSQP')

x = np.linspace(0, 3, len(result.x))

plt.plot(x, result.x)
plt.show()

