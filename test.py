import math
import numpy as np
import CustomFunction.CustomFunction as cf

q0 = [0, 0, 0, 1]
q1 = [0.2, 0.1, 0.1, 0.6]
q2 = [0.1, 0.2, 0.1, 0.6]

# print(np.dot(cf.Convert2Matrix_Quaternion(q1), (np.dot(cf.Convert2Matrix_Quaternion(q2), q0))))
# print(np.dot(cf.Convert2Matrix_Quaternion(q2), (np.dot(cf.Convert2Matrix_Quaternion(q1), q0))))

e0 = [0, 0, 0]
e1 = [10, 30, 70]
e2 = [4, 65, 34]

def e2q(e):
    roll, pitch, yaw = np.deg2rad(e[0]), np.deg2rad(e[1]), np.deg2rad(e[2])
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    q_norm = np.linalg.norm([qx, qy, qz, qw])
    qx_normalized = qx / q_norm
    qy_normalized = qy / q_norm
    qz_normalized = qz / q_norm
    qw_normalized = qw / q_norm

    return [qx_normalized, qy_normalized, qz_normalized, qw_normalized]

def q2e(q):
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    pitch = math.asin(2 * (qw * qy - qz * qx))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

    return [np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)]



# print(e2q(e1), cf.Euler2Quaternion(e1))
# print(q2e(q1), cf.Quaternion2Euler(q1))
# print(q2e(e2q(e1)))

q_zero = [0, 0, 0, 1]
q_init = [0, 0.2, 0, 0.8]
# print(np.dot(cf.Convert2Matrix_Quaternion(q_init, inverse = True), q_zero))
q_robot = [0.3, 0.1, 0.2, 0.4]
# q = [0.25, 0.05, 0.25, 0.35]

q1 = [0.29742345368, 0.132432784638, 0.14238732468, 0.62344279462]
q2 = [0.11043288364, 0.223474944978, 0.11234286349, 0.62134143323]
# q = np.dot(cf.Convert2Matrix_Quaternion(q_init), np.dot(cf.Convert2Matrix_Quaternion(q_robot, inverse=True), q_zero))
# print(np.dot(np.dot(q_init, cf.Convert2Matrix_Quaternion(q_robot, inverse=True)), cf.Convert2Matrix_Quaternion(q_robot)))
# print(np.dot(cf.Convert2Matrix_Quaternion(q), q_robot))
# print(np.dot(q, cf.Convert2Matrix_Quaternion(q_robot)))

weight_list = np.linspace(0, 1, 500)
q_list = []
np.set_printoptions(precision=15)
for weight in weight_list:
    q_list.append(cf.Slerp_Quaternion(q2, q1, weight))
    print(cf.Slerp_Quaternion(q2, q1, weight))