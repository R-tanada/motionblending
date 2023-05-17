import CustomFunction.Calculation as cf
import numpy as np

q_init = [0, 0, 0, 1]
q_1 = [0.01, 0.01, 0.01, 0.97]

matrix = cf.Convert2Matrix_Quaternion(q_1)
q_2 = np.dot(matrix, q_init)
print(q_2)
