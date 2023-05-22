import numpy as np
from matplotlib import pyplot as plt

tn = 2
tf = 2.5
pos_n = [30, 50, 40]
pos_f = [50, 20, 58]
vel_n = [3, -30, 1]
vel_f = [0, 0, 0]
acc_n = [0.5, -5, 0.5]
acc_f = [0, 0, 0]
loopCount = 100

a_matrix = [
    [1, tn, tn**2,  tn**3,      tn**4,      tn**5       ],
    [0, 1,  2*tn,   3*(tn**2),  4*(tn**3),  5*(tn**4)   ],
    [0, 0,  2,      6*tn,       12*(tn**2), 20*(tn**3)  ],
    [1, tf, tf**2,  tf**3,      tf**4,      tf**5       ],
    [0, 1,  2*tf,   3*(tf**2),  4*(tf**3),  5*(tf**4)   ],
    [0, 0,  2,      6*tf,       12*(tf**2), 20*(tf**3)  ]
]
b_matrix = [pos_n, vel_n, acc_n, pos_f, vel_f, acc_f]
coeff = np.linalg.solve(a_matrix, b_matrix)

def function(coeff, x):
    return coeff[0] + coeff[1]*x + coeff[2]*(x**2) + coeff[3]*(x**3) + coeff[4]*(x**4) + coeff[5]*(x**5)

flameLength = (loopCount/ tn) * (tf - tn)
flame = np.linspace(tn, tf, int(flameLength))

print(coeff[0])
print(coeff[:, 0])
y = []
for i in range(3):
    y.append(function(coeff[:, i], flame))
# y = function(coeff, flame)
# print(function(coeff[:, 0], flame))

print(np.transpose(y))

# plt.plot(flame, y[1])
# plt.show()