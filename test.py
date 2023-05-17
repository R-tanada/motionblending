import math
import numpy as np
import CustomFunction.CustomFunction as cf

def Slerp_Quaternion(Quaternion, initQuaternion, weight):
    e = 0.0000001
    dot = np.dot(initQuaternion, Quaternion)
    if dot > 1:
        dot = 1
    elif dot < -1:
        dot = -1
    theta = math.acos(dot)
    return (math.sin((1 - weight) * theta)/ (math.sin(theta) + e)) * np.array(initQuaternion) + (math.sin(weight * theta)/ (math.sin(theta) + e)) * np.array(Quaternion)

q1 = np.array([0.15, 0.8, 0.025, 0.025])
q2 = np.array([0.8, -0.2, 0.2, 0.2])
weight_list = np.linspace(0, 1, 5)

# print(Slerp_Quaternion(q2, q1, 0.5))

q = []
for weight in weight_list:  
    q.append(Slerp_Quaternion(q2, q1, weight))

# print(q[1])

q3 = [179.9, -32, 0]
print(cf.Quaternion2Euler(cf.Euler2Quaternion(q3)))

e1 = [0, 0, 0]
print(cf.Euler2Quaternion(e1))


