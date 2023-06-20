import pybullet as p
import time
import pybullet_data as pd
import numpy as np
import json
import math
import CustomFunction as cf

class SimulationManager():
    def __init__(self, configs: dict) -> None:
        self.SetEnviroment()
        self.CreateBody()
        self.robotManager = {}
        for arm in configs.keys():
            self.robotManager[configs[arm]['Mount']] = RobotManager(configs[arm])
        self.SetInitTransform()
        

    def SetEnviroment(self):
        p.connect(p.GUI)
        # p.setViewport(0, 0, 1920, 1080)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        planeId = p.loadURDF("/Users/yuzu/envs/simu/lib/python3.8/site-packages/pybullet_data/plane.urdf")
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # GUI表示を無効化
        p.setAdditionalSearchPath(pd.getDataPath())

        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-45, cameraTargetPosition=[0.5, 0, 0.5])

        p.setTimeStep(1/200)
        p.setGravity(0,0,-9.8)

    def CreateBody(self):
        halfExtents = [0.143, 0.143, 0.61]  # 正方形の各辺の長さの半分
        startPos = [0, 0, 0.61]  # オブジェクトの起点位置（x, y, z）
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])  # オブジェクトの起点姿勢（オイラー角）
        boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)

        visualShapeId = p.createVisualShape(p.GEOM_BOX, halfExtents=halfExtents, rgbaColor=[0.75, 0.75, 0.75, 1.0])
        multiBodyId = p.createMultiBody(baseCollisionShapeIndex=boxId, baseVisualShapeIndex=visualShapeId, basePosition=startPos, baseOrientation=startOrientation)

        halfExtents2 = [0.303, 0.303, 0.1]  # 正方形の各辺の長さの半分
        startPos2 = [0, 0, 0.1]  # オブジェクトの起点位置（x, y, z）
        startOrientation2 = p.getQuaternionFromEuler([0, 0, 0])  # オブジェクトの起点姿勢（オイラー角）
        boxId2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents2)

        visualShapeId2 = p.createVisualShape(p.GEOM_BOX, halfExtents=halfExtents2, rgbaColor=[0.75, 0.75, 0.75, 1.0])
        multiBodyId2 = p.createMultiBody(baseCollisionShapeIndex=boxId2, baseVisualShapeIndex=visualShapeId2, basePosition=startPos2, baseOrientation=startOrientation2)


    def SendDataToRobot(self, motions: dict):
        for mount in motions.keys():
            self.robotManager[mount].SendDataToRobot(motions[mount])

        # p.stepSimulation()

    def SetInitTransform(self):
        for mount in self.robotManager.keys():
            self.robotManager[mount].SetInitTransform()

class RobotManager():
    def __init__(self, config) -> None:
        basePosition, baseRotation = self.GetBasePositionRotation(config['Mount'])
        self.arm = p.loadURDF("/Users/yuzu/Desktop/simulation/urdf/xarm7.urdf", basePosition, baseRotation, useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        self.homoMatrix, self.InverseMatrix = self.SetTransformMatrix(basePosition, baseRotation)
        self.initPos = np.array(config['InitPos'])/1000
        # self.SetInitTransform()

    def SetInitTransform(self):
        for i in range(100):
            self.SendDataToRobot(self.initPos, [0, 0, 0, 1], 850)
            time.sleep(0.01)

    def SetTransformMatrix(self, position, rotation):
        matrix = list(p.getMatrixFromQuaternion(rotation))
        matrix = np.array([matrix[0:3],
                                        matrix[3:6],
                                        matrix[6:9]])
        matrix = np.append(matrix, np.array([[0, 0, 0]]), axis = 0)
        transform_matrix = np.append(matrix, np.hstack((position, 1)).reshape(1, 4).transpose(), axis=1)

        initInverseMatrix = cf.Convert2InverseMatrix(rotation)

        return transform_matrix, initInverseMatrix
    
    def GetBasePositionRotation(self, mount):
        if mount == 'right':
            basePosition = [0, -0.143, 1.05]
            baseRotation = p.getQuaternionFromEuler([math.pi/2, 0, 0])

        elif mount == 'left':
            basePosition = [0, 0.143, 1.05]
            baseRotation = p.getQuaternionFromEuler([-math.pi/2, 0, 0])

        return basePosition, baseRotation
    
    def SendDataToRobot(self, position, rotation, gripper):
        position = np.dot(self.homoMatrix, np.hstack((position, 1)))[0:3]
        rotation = (np.dot(self.InverseMatrix, rotation))
        gripper = gripper

        self.InverseKinematics(position, rotation)

        p.stepSimulation()
    
    def InverseKinematics(self, position, rotation, nullspace: bool = True, usedynamics: bool = True, maxiter: int = 50):
        xarmNumDofs = 7 

        ll = [-17]*xarmNumDofs
        #upper limits for null space (todo: set them to proper range)
        ul = [17]*xarmNumDofs
        #joint ranges for null space (todo: set them to proper range)
        jr = [17]*xarmNumDofs
        #restposes for null space

        jointPoses = [0] * 7
        
        if nullspace:
            restPoses = [0]*xarmNumDofs
            jointPoses = p.calculateInverseKinematics(self.arm, 7, position, rotation, lowerLimits=ll, 
            upperLimits=ul,jointRanges=jr, restPoses=np.array(restPoses).tolist(),residualThreshold=1e-5, maxNumIterations=maxiter)
            self.jointPoses = jointPoses
        else:
            self.jointPoses = p.calculateInverseKinematics(self.arm, 7, position, rotation, maxNumIterations=maxiter)

        if usedynamics:
            for i in range(xarmNumDofs):
                pose = self.jointPoses[i]
                p.setJointMotorControl2(self.arm, i+1, p.POSITION_CONTROL, pose,force=5 * 240.)
        else:
            for i in range(xarmNumDofs):
                p.resetJointState(self.arm, i+1, jointPoses[i])

if __name__ == '__main__':
    with open('SettingFile/settings_dual.json', 'r') as file:
        setting = json.load(file)

    simu = SimulationManager(setting['xArmConfigs'])

    while True:
        time.sleep(0.3)