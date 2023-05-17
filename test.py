import math
import numpy as np

def Slerp_Quaternion(Quaternion, initQuaternion, weight):
    e = 0.0000001
    dot = np.dot(initQuaternion, Quaternion)
    if dot > 1:
        dot = 1
    elif dot < -1:
        dot = -1
    theta = math.acos(dot)
    return (math.sin((1 - weight) * theta)/ (math.sin(theta) + e)) * np.array(initQuaternion) + (math.sin(weight * theta)/ (math.sin(theta) + e)) * np.array(Quaternion)

q1 = [0, 0, 0, 1]
q2 = [0.2, 0.2, 0.2, 0.4]
weight_list = np.linspace(0, 1, 300)

q = []
for weight in weight_list:  
    q.append(Slerp_Quaternion(q2, q1, weight))

print(q)