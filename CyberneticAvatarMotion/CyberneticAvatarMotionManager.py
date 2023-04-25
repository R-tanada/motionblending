from 
import numpy as np
import scipy.spatial.transform as scitransform
import quaternion
import math

class CyberneticAvatarMotionManager:

    originPositions     = {}
    inversedMatrixforPosition ={}
    inversedMatrix      = {}

    beforePositions     = {}
    weightedPositions   = {}

    beforeRotations     = {}
    weightedRotations   = {}

    def __init__(self, ParticipantConfigs) -> None:
        for i in range(defaultParticipantNum):
            self.originPositions['participant'+str(i+1)] = np.zeros(3)
            self.inversedMatrixforPosition['participant'+str(i+1)] = np.array([[1,0,0],[0,1,0],[0,0,1]])
            self.inversedMatrix['participant'+str(i+1)] = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
            
            self.beforePositions['participant'+str(i+1)] = np.zeros(3)
            self.weightedPositions['participant'+str(i+1)] = np.zeros(3)

            self.beforeRotations['participant'+str(i+1)] = np.array([0,0,0,1])
            self.weightedRotations['participant'+str(i+1)] = np.array([0,0,0,1])
        
        self.participantNum = defaultParticipantNum
        self.before_position = [[0, 0, 0],[0, 0, 0]]
        self.customweightPosition = [0, 0, 0]
        self.before_sharedPosition = [0, 0, 0]

    def GetSharedTransform(self, position: dict, rotation: dict, weight: list):

        participantMotions = [
            {
            'right': {'position': [1, 1, 1], 'rotation': [1, 1, 1], 'gripper': 5}, 
            'left': {'position': [1, 1, 1], 'rotation': [1, 1, 1], 'gripper': 5}
            }, 
            {
            'right': {'position': [1, 1, 1], 'rotation': [1, 1, 1], 'gripper': 5}
            }
        ]

        pos = self.GetRelativePosition(position)
        rot = self.GetRelativeRotation(rotation)

        self.ConvetAxis()

        # ----- Shared transform ----- #
        sharedPosition = [0, 0, 0]
        sharedRotation_euler = [0, 0, 0]

        for i in range(self.participantNum):
            # ----- Position ----- #
            diffPos     = pos['participant'+str(i+1)] - self.beforePositions['participant'+str(i+1)]
            weightedPos = diffPos * weight[0][i] + self.weightedPositions['participant'+str(i+1)]
            sharedPosition += weightedPos

            self.weightedPositions['participant'+str(i+1)]  = weightedPos
            self.beforePositions['participant'+str(i+1)]    = pos['participant'+str(i+1)]

            # ----- Rotation ----- #
            qw, qx, qy, qz = self.beforeRotations['participant'+str(i+1)][3], self.beforeRotations['participant'+str(i+1)][0], self.beforeRotations['participant'+str(i+1)][1], self.beforeRotations['participant'+str(i+1)][2]
            mat4x4 = np.array([ [qw, qz, -qy, qx],
                                [-qz, qw, qx, qy],
                                [qy, -qx, qw, qz],
                                [-qx,-qy, -qz, qw]])
            currentRot = rot['participant'+str(i+1)]
            diffRot = np.dot(np.linalg.inv(mat4x4), currentRot)
            diffRotEuler = self.Quaternion2Euler(np.array(diffRot))
            
            weightedDiffRotEuler = list(map(lambda x: x * weight[1][i] , diffRotEuler))
            weightedDiffRot = self.Euler2Quaternion(np.array(weightedDiffRotEuler))

            nqw, nqx, nqy, nqz = weightedDiffRot[3], weightedDiffRot[0], weightedDiffRot[1], weightedDiffRot[2]
            neomat4x4 = np.array([[nqw, -nqz, nqy, nqx],
                                    [nqz, nqw, -nqx, nqy],
                                    [-nqy, nqx, nqw, nqz],
                                    [-nqx,-nqy, -nqz, nqw]])
            weightedRot = np.dot(neomat4x4,  self.weightedRotations['participant'+str(i+1)])
            sharedRotation_euler += self.Quaternion2Euler(weightedRot)

            self.weightedRotations['participant'+str(i+1)]  = weightedRot
            self.beforeRotations['participant'+str(i+1)]    = rot['participant'+str(i+1)]

        return sharedPosition, sharedRotation_euler
    
    def IntegrationWithWeight(self):
        
    
    def ConvertAxis(self, positon, rotation, mount):
        if mount == 'virtical':
            position = [position[0], positon[1], positon[2]]
            rotation = [rotation[0], rotation[1], rotation[2], rotation[3]]

        elif mount == 'right':
            position = [position[0], positon[1], positon[2]]
            rotation = [rotation[0], rotation[1], rotation[2], rotation[3]]
 
        elif mount == 'left':
            position = [position[0], positon[1], positon[2]]
            rotation = [rotation[0], rotation[1], rotation[2], rotation[3]]


    def SetInitPosition(self, position) -> None:
        # ----- numpy array to dict: position ----- #
        if type(position) is np.ndarray:
            position = self.NumpyArray2Dict(position)
        
        #print(position)

        listParticipant = [participant for participant in list(position.keys()) if 'participant' in participant]
        self.participantNum = len(listParticipant)
        
        for i in range(self.participantNum):
            self.originPositions['participant'+str(i+1)] = position['participant'+str(i+1)]
        
    def GetRelativePosition(self, position): 
        relativePos = {}
        for i in range(self.participantNum):
            relativePos['participant'+str(i+1)] = position['participant'+str(i+1)] - self.originPositions['participant'+str(i+1)]
        
        return relativePos
    

    def SetInversedMatrix(self, rotation) -> None:
        listParticipant = [participant for participant in list(rotation.keys()) if 'participant' in participant]
        self.participantNum = len(listParticipant)

        for i in range(self.participantNum):
            q = rotation['participant'+str(i+1)]
            qw, qx, qy, qz = q[3], q[1], q[2], q[0]
            mat4x4 = np.array([ [qw, -qy, qx, qz],
                                [qy, qw, -qz, qx],
                                [-qx, qz, qw, qy],
                                [-qz,-qx, -qy, qw]])
            self.inversedMatrix['participant'+str(i+1)] = np.linalg.inv(mat4x4)

    def GetRelativeRotation(self, rotation):
        relativeRot = {}
        for i in range(self.participantNum):
            relativeRot['participant'+str(i+1)] = np.dot(self.inversedMatrix['participant'+str(i+1)], rotation['participant'+str(i+1)])

        return relativeRot

    def Quaternion2Euler(self, q, isDeg: bool = True):
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qw = q[3]

        m00 = 1 - (2 * qy**2) - (2 * qz**2)
        m01 = (2 * qx * qy) + (2 * qw * qz)
        m02 = (2 * qx * qz) - (2 * qw * qy)
        m10 = (2 * qx * qy) - (2 * qw * qz)
        m11 = 1 - (2 * qx**2) - (2 * qz**2)
        m12 = (2 * qy * qz) + (2 * qw * qx)
        m20 = (2 * qx * qz) + (2 * qw * qy)
        m21 = (2 * qy * qz) - (2 * qw * qx)
        m22 = 1 - (2 * qx**2) - (2 * qy**2)

        if m02 == 1:
            tx = math.atan2(m10, m11)
            ty = math.pi/2
            tz = 0
        elif m02 == -1:
            tx = math.atan2(m21, m20)
            ty = -math.pi/2
            tz = 0
        else:
            tx = -math.atan2(-m12, m22)
            ty = -math.asin(m02)
            tz = -math.atan2(-m01, m00)

        if isDeg:
            tx = np.rad2deg(tx)
            ty = np.rad2deg(ty)
            tz = np.rad2deg(tz)

        rotEuler = np.array([tx, ty, tz])
        return rotEuler